#!/usr/bin/env bash
set -e

VENV=${SPNET_VENV:-$HOME/Software/spnet-venv}
cd "$(dirname "$0")"

echo ">>> Export ONNX (512x640)"
"$VENV/bin/python" export_onnx_512_640.py

echo ">>> Export ONNX (480x640)"
"$VENV/bin/python" export_onnx_480_640.py

echo ">>> All ONNX exports finished successfully."
