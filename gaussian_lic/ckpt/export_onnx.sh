#!/usr/bin/env bash
# Exports SPNet to ONNX: spnet_512_640.onnx and spnet_480_640.onnx, or the sizes given (e.g. 480_640).
# Downloads the checkpoint Large_300.pth first when it is not here.
set -e

VENV=${SPNET_VENV:-$HOME/Software/spnet-venv}
cd "$(dirname "$0")"

if [ ! -f Large_300.pth ]; then
    echo ">>> Downloading Large_300.pth"
    "$VENV/bin/gdown" 11dujPviL4pKLEXytXK0mEmPBNQDqgEak -O Large_300.pth
fi

for size in ${@:-512_640 480_640}; do
    echo ">>> Export ONNX (${size/_/x})"
    "$VENV/bin/python" export_onnx_$size.py
done

echo ">>> All ONNX exports finished successfully."
