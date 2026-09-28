"""Export one trusted ProtoMotions SOMA23 .motion to portable numeric NPZ.

Run this in the original ProtoMotions Python environment (with its package on
PYTHONPATH). No calibration-viewer installation is needed in that environment.
"""
from pathlib import Path
import argparse

import numpy as np


def export_motion(source: Path, destination: Path) -> None:
    import torch
    data = torch.load(source, map_location="cpu", weights_only=False)
    if not isinstance(data, dict) or "rigid_body_pos" not in data:
        raise ValueError("Expected a single ProtoMotions SOMA23 motion dictionary")
    value = data["rigid_body_pos"]
    points = value.detach().cpu().numpy() if hasattr(value, "detach") else np.asarray(value)
    if points.ndim != 3 or points.shape[1:] != (23, 3) or not len(points) or not np.isfinite(points).all():
        raise ValueError("Expected finite SOMA23 rigid_body_pos [T,23,3] in MJCF body order")
    fps = float(data["fps"])
    if not np.isfinite(fps) or fps <= 0:
        raise ValueError("fps must be positive")
    if destination.suffix.lower() != ".npz":
        raise ValueError("Destination must end in .npz")
    destination.parent.mkdir(parents=True, exist_ok=True)
    # Raw rigid_body_rot is a SOMA joint-frame orientation, not canonical heading.
    np.savez_compressed(destination, rigid_body_pos=points, fps=fps)
    print(f"Saved {len(points)} SOMA23 frames at {fps:g} fps to {destination}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    export_motion(args.input, args.output)
