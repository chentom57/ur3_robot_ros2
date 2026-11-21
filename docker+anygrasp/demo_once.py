#!/usr/bin/env python3
"""Run one SDK default inference on official example data; save demo grasps."""
import argparse
import os
from pathlib import Path
import runpy
import sys


def run_once(sdk_dir, checkpoint, output, width=0.085, height=0.03):
    import numpy as np
    import torch

    sdk_dir = Path(sdk_dir).resolve()
    checkpoint, output = Path(checkpoint).resolve(), Path(output).resolve()
    if not checkpoint.is_file():
        raise FileNotFoundError(checkpoint)
    if not torch.cuda.is_available():
        raise RuntimeError('CUDA GPU is unavailable')
    previous_dir, previous_args, previous_path = Path.cwd(), sys.argv[:], sys.path[:]
    sys.path.insert(0, str(sdk_dir))
    try:
        os.chdir(sdk_dir)
        # Load official helper functions without executing demo.py's multi-demo main.
        sys.argv = ['demo.py', '--checkpoint_path', str(checkpoint),
                    '--max_gripper_width', str(width), '--gripper_height', str(height)]
        demo = runpy.run_path(str(sdk_dir / 'demo.py'), run_name='sdk_demo_helpers')
        with torch.inference_mode():
            detector = demo['create_detector'](demo['cfgs'])
            if not detector:
                raise RuntimeError('Detector creation failed; check license and checkpoint')
            points, _, _ = demo['load_point_cloud'](str(sdk_dir / 'example_data'))
            grasps = demo['predict_with_configs'](
                detector, points, dense_grasp=False, collision_detection=True)
        if grasps is None or len(grasps) == 0:
            raise RuntimeError('No grasp candidates survived filtering')
        rows = np.asarray(grasps.grasp_group_array)
        if rows.ndim != 2 or rows.shape[1] != 17 or not np.isfinite(rows).all():
            raise RuntimeError('Unexpected GraspGroup output; nothing saved')
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open('wb') as stream:
            np.save(stream, rows, allow_pickle=False)
        print(f'Single inference passed: {len(rows)} candidates', flush=True)
        print(f'Official example-data output: {output}', flush=True)
        print('These candidates belong to the SDK example scene, not the live robot scene.', flush=True)
    finally:
        sys.argv = previous_args
        os.chdir(previous_dir)
        sys.path[:] = previous_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdk-dir', default=str(Path(__file__).resolve().parent))
    parser.add_argument('--checkpoint_path', default='log/checkpoint_detection.tar')
    parser.add_argument('--max_gripper_width', type=float, default=0.085)
    parser.add_argument('--gripper_height', type=float, default=0.03)
    parser.add_argument('--output', default='results/official_demo_grasps.npy')
    args = parser.parse_args()
    if not 0 < args.max_gripper_width <= 0.1:
        parser.error('--max_gripper_width must be in (0, 0.1] meters')
    if not 0 < args.gripper_height < float('inf'):
        parser.error('--gripper_height must be positive and finite')
    sdk = Path(args.sdk_dir).resolve()
    checkpoint = Path(args.checkpoint_path)
    output = Path(args.output)
    run_once(sdk, checkpoint if checkpoint.is_absolute() else sdk / checkpoint,
             output if output.is_absolute() else sdk / output,
             args.max_gripper_width, args.gripper_height)


if __name__ == '__main__':
    main()
