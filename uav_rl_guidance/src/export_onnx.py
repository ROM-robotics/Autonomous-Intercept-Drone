#!/usr/bin/env python3
"""Export policy.pt (TorchScript GRU) to policy.onnx for ONNX Runtime C++ inference.

Usage:
    cd uav_rl_guidance
    python3 src/export_onnx.py

This replaces the extract_weights.py + Eigen GRU approach.
The exported .onnx is loaded directly by rl_guidance_node.cpp via Ort::Session.
"""

import os
import sys

import torch


MODEL_DIR = os.path.join(os.path.dirname(__file__), "..", "models")
MODEL_PATH = os.path.join(MODEL_DIR, "policy.pt")
OUTPUT_PATH = os.path.join(MODEL_DIR, "policy.onnx")


def main():
    if not os.path.exists(MODEL_PATH):
        print(f"ERROR: model not found at {MODEL_PATH}", file=sys.stderr)
        sys.exit(1)

    print(f"Loading {MODEL_PATH} ...")
    m = torch.jit.load(MODEL_PATH, map_location="cpu")
    m.eval()

    # Dummy inputs for tracing
    obs = torch.zeros(1, 15)
    h = torch.zeros(1, 1, 128)

    print("Exporting to ONNX ...")
    torch.onnx.export(
        m,
        (obs, h),
        OUTPUT_PATH,
        input_names=["obs", "h_in"],
        output_names=["action", "h_out"],
        opset_version=14,
        dynamo=False,
    )

    size_kb = os.path.getsize(OUTPUT_PATH) / 1024
    print(f"✓ Written {OUTPUT_PATH}  ({size_kb:.0f} KB)")

    # Quick sanity check
    import onnxruntime as ort
    import numpy as np
    sess = ort.InferenceSession(OUTPUT_PATH)
    obs_np = np.random.randn(1, 15).astype(np.float32)
    h_np = np.zeros((1, 1, 128), dtype=np.float32)
    out = sess.run(None, {"obs": obs_np, "h_in": h_np})
    assert out[0].shape == (1, 4), f"Bad action shape: {out[0].shape}"
    assert out[1].shape == (1, 1, 128), f"Bad hidden shape: {out[1].shape}"
    print("✓ ONNX Runtime inference verified")


if __name__ == "__main__":
    main()
