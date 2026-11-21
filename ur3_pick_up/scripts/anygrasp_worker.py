#!/usr/bin/env python3
"""One live-cloud request in the licensed SDK container; no robot/ROS imports."""
import argparse
import json
import os
from pathlib import Path
import sys


def infer(args):
    import numpy as np
    import torch
    sdk = Path(args.sdk_dir).resolve()
    os.chdir(sdk)  # SDK resolves its license relative to this directory.
    sys.path.insert(0, str(sdk))
    from gsnet import create_detector
    with np.load(args.input, allow_pickle=False) as data:
        points = np.asarray(data['points'], dtype=np.float32)
        region = data['region']
        request_id = str(data['request_id'].item())
        frame_id = str(data['frame_id'].item())
        stamp = data['stamp'].tolist()
    if (points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all()
            or region.dtype != np.bool_ or region.shape != (len(points),) or not region.any()):
        raise ValueError('Invalid points / empty ROI mask; inference refused')
    if not torch.cuda.is_available():
        raise RuntimeError('CUDA GPU unavailable')
    config = argparse.Namespace(checkpoint_path=args.checkpoint,
        max_gripper_width=args.width, gripper_height=args.height)
    with torch.inference_mode():
        detector = create_detector(config)
        if detector is None:
            raise RuntimeError('Detector initialization failed; check license and checkpoint')
        grasps = detector.get_grasp(points, {'dense_grasp': False,
            'collision_detection': True, 'region_steering': region})
        if grasps is None or len(grasps) == 0:
            raise RuntimeError('AnyGrasp produced no collision-filtered candidates')
        rows = np.asarray(grasps.nms().sort_by_score().grasp_group_array)
    if rows.ndim != 2 or rows.shape[1] != 17 or not np.isfinite(rows).all():
        raise ValueError('Invalid GraspGroup output')
    candidates = [dict(score=float(r[0]), width=float(r[1]), height=float(r[2]), depth=float(r[3]),
        rotation_matrix=r[4:13].reshape(3, 3).tolist(), translation=r[13:16].tolist()) for r in rows]
    output = Path(args.output)
    temporary = output.with_suffix('.tmp')
    temporary.write_text(json.dumps(dict(request_id=request_id, frame_id=frame_id,
        stamp=dict(sec=int(stamp[0]), nanosec=int(stamp[1])), candidates=candidates), allow_nan=False))
    temporary.replace(output)
    print(f'Live inference completed: {len(candidates)} candidates', flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--sdk-dir', required=True)
    p.add_argument('--checkpoint', required=True)
    p.add_argument('--width', type=float, required=True)
    p.add_argument('--height', type=float, required=True)
    args = p.parse_args()
    # Host can terminate precisely this process if docker exec is interrupted.
    Path(args.input).with_suffix('.pid').write_text(str(os.getpid()))
    infer(args)


if __name__ == '__main__':
    main()
