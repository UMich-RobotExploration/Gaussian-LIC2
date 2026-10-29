#!/usr/bin/env bash
set -e

VENV=${SPNET_VENV:-$HOME/Software/spnet-venv}

echo ">>> Creating Python venv: $VENV"
# --without-pip + the system pip, so python3-venv does not need to be installed
python3 -m venv --without-pip --system-site-packages "$VENV"

echo ">>> Installing PyTorch (CUDA 12.8 build, needed for RTX 50xx GPUs)"
"$VENV/bin/python" -m pip install torch==2.7.1 torchvision==0.22.1 --index-url https://download.pytorch.org/whl/cu128

echo ">>> Installing ONNX and gdown (downloads the checkpoint from Google Drive)"
"$VENV/bin/python" -m pip install onnx gdown

echo ">>> Done. Venv $VENV is ready."
