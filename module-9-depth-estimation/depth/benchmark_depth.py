"""Compare depth models and input sizes on one photo: speed and the distances they report.

    python3 /workspace/depth/benchmark_depth.py
    python3 /workspace/depth/benchmark_depth.py --sizes small base --inputs 266 364 518
    python3 /workspace/depth/benchmark_depth.py --image my_photo.jpg --target "red ball"

By default it uses the newest photo saved by the depth node. The model size "base"
downloads 390 MB the first time and "large" 1.3 GB.
"""

import argparse
import glob
import os
import time

from PIL import Image

import depth_estimator

RESULTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results", "images")


def newest_photo():
    photos = [p for p in glob.glob(os.path.join(RESULTS, "*.jpg")) if not p.endswith("_depth.jpg")]
    if not photos:
        raise SystemExit("No photo found. Pass one with --image, or ask the depth node a question first.")
    return max(photos, key=os.path.getmtime)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--image", default=None, help="photo to use (default: newest saved photo)")
    parser.add_argument("--sizes", nargs="+", default=["small"], choices=list(depth_estimator.MODELS))
    parser.add_argument("--inputs", nargs="+", type=int, default=[364], help="input sizes in pixels")
    parser.add_argument("--runs", type=int, default=3, help="timed runs per setting")
    parser.add_argument("--target", default="", help="also report the distance to this object")
    args = parser.parse_args()

    path = args.image or newest_photo()
    image = Image.open(path).convert("RGB")
    import torch
    print(f"Photo: {path} ({image.width}x{image.height}), CPU threads: {torch.get_num_threads()}")
    print("model   input px   load (s)   per photo (s)   ahead (m)   target (m)")
    for size in args.sizes:
        start = time.time()
        estimator = depth_estimator.DepthEstimator(size, args.inputs[0])
        load = time.time() - start
        for pixels in args.inputs:
            estimator.input_size = pixels
            estimator.estimate(image)                        # warm-up run, not timed
            start = time.time()
            for _ in range(args.runs):
                depth = estimator.estimate(image)
            per_photo = (time.time() - start) / args.runs
            answer = depth_estimator.analyze(depth, image, args.target)
            target = (f"{answer['target_raw_m']:.2f}" if answer.get("target_found")
                      else ("not found" if args.target else "-"))
            print(f"{size:<7} {pixels:>8} {load:>10.1f} {per_photo:>15.2f} "
                  f"{answer['ahead_raw_m']:>11.2f} {target:>12}")
        del estimator


if __name__ == "__main__":
    main()
