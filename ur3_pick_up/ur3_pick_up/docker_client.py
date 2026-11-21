"""File-based, per-request transport into the existing fixed-MAC SDK container."""
import json
from pathlib import Path
import subprocess
import uuid

import numpy as np
from .anygrasp_input import load_candidates


class DockerClient:
    def __init__(self, settings, worker):
        self.s = settings
        self.worker = Path(worker)
        self.prefix = (['sudo', '-n'] if settings['docker_use_sudo'] else []) + ['docker']
        self.container = settings['docker_container']

    def command(self, args, timeout=20):
        try:
            result = subprocess.run(self.prefix + list(args), text=True,
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout)
        except subprocess.TimeoutExpired as exc:
            raise TimeoutError(f'Docker command timed out after {timeout}s') from exc
        if result.returncode:
            raise RuntimeError(f'Docker failed: {result.stdout[-2500:]}\n'
                'If sudo needs authentication, run sudo -v in this terminal before starting pick_up.')
        return result.stdout

    def preflight(self):
        status = self.command(['inspect', '--format', '{{.State.Running}}', self.container]).strip()
        if status == 'false':
            self.command(['start', self.container])
        elif status != 'true':
            raise RuntimeError(f'Unexpected container state: {status}')
        # No container recreation or network changes: preserve licensed identity.
        self.command(['exec', self.container, 'python', '-c',
            'import pathlib,sys; p=pathlib.Path(sys.argv[1]); '
            'assert p.is_file(), str(p); '
            'assert (pathlib.Path(sys.argv[2])/"gsnet.so").is_file(), "gsnet.so missing"',
            self.s['checkpoint_path'], self.s['docker_sdk_dir']])

    def cancel_worker(self, remote):
        script = ('import os,signal,pathlib,sys; '
                  'p=pathlib.Path(sys.argv[1]); '
                  'pid=int(p.read_text()) if p.exists() else 0; '
                  'os.kill(pid,signal.SIGTERM) if pid>1 else None')
        try:
            self.command(['exec', self.container, 'python', '-c', script, remote + '/cloud.pid'], timeout=5)
        except (OSError, RuntimeError, TimeoutError):
            pass

    def infer(self, points, region, frame_id, stamp, base_from_camera, width):
        request_id = uuid.uuid4().hex
        job = Path(self.s['exchange_dir']).expanduser() / request_id
        job.mkdir(parents=True, exist_ok=False)
        remote = '/tmp/ur3_anygrasp_' + request_id
        np.savez_compressed(job / 'cloud.npz', points=points, region=region,
            request_id=np.array(request_id), frame_id=np.array(frame_id), stamp=np.array(stamp, dtype=np.int64))
        (job / 'capture.json').write_text(json.dumps(dict(base_from_camera=base_from_camera.tolist(),
            workspace_min=self.s['workspace_min'], workspace_max=self.s['workspace_max'])))
        print(f'Point cloud / inference records: {job}', flush=True)
        self.command(['exec', self.container, 'mkdir', '-p', remote])
        try:
            self.command(['cp', str(job / 'cloud.npz'), self.container + ':' + remote + '/cloud.npz'])
            self.command(['cp', str(self.worker), self.container + ':' + remote + '/worker.py'])
            args = ['exec', '-w', self.s['docker_sdk_dir'], '-e',
                'PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True', self.container, 'python',
                remote + '/worker.py', '--input', remote + '/cloud.npz', '--output', remote + '/grasps.json',
                '--sdk-dir', self.s['docker_sdk_dir'], '--checkpoint', self.s['checkpoint_path'],
                '--width', str(width), '--height', str(self.s['gripper_height'])]
            with (job / 'inference.log').open('w') as log:
                process = subprocess.Popen(self.prefix + args, stdout=log, stderr=subprocess.STDOUT)
                try:
                    process.wait(timeout=self.s['inference_timeout'])
                except (subprocess.TimeoutExpired, KeyboardInterrupt):
                    self.cancel_worker(remote)
                    process.terminate()
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill(); process.wait()
                    raise
            if process.returncode:
                raise RuntimeError(f'AnyGrasp inference failed; {job / "inference.log"}\n'
                                   + (job / 'inference.log').read_text()[-2500:])
            self.command(['cp', self.container + ':' + remote + '/grasps.json', str(job / 'grasps.json')])
            metadata = json.loads((job / 'grasps.json').read_text())
            if metadata.get('request_id') != request_id:
                raise ValueError('Inference response request_id mismatch')
            candidates, result_frame, result_stamp = load_candidates(job / 'grasps.json')
            if result_frame != frame_id or result_stamp != tuple(stamp):
                raise ValueError('Inference response capture frame/time mismatch')
            return candidates
        except subprocess.TimeoutExpired as exc:
            raise TimeoutError(f'AnyGrasp inference exceeded {self.s["inference_timeout"]}s; see {job}') from exc
        finally:
            # Only remove this UUID job, never SDK/license/container data.
            try:
                self.command(['exec', self.container, 'rm', '-rf', remote], timeout=5)
            except (OSError, RuntimeError, TimeoutError):
                pass
