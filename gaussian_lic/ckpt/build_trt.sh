#!/usr/bin/env bash
set -e

TENSORRT_ROOT=${TENSORRT_ROOT:-$HOME/Software/TensorRT-10.16.1.11}
CUDA_ROOT=${CUDA_ROOT:-$HOME/Software/cuda-12.8}
TRT_BIN=$TENSORRT_ROOT/bin/trtexec
cd "$(dirname "$0")"
ONNX=${ONNX:-spnet_480_640.onnx}  # exported model, see export_onnx.sh
ENGINE_DIR=${ENGINE_DIR:-.}

echo ">>> Setting TensorRT / CUDA LD_LIBRARY_PATH"
export LD_LIBRARY_PATH=$TENSORRT_ROOT/lib:$CUDA_ROOT/lib64:$LD_LIBRARY_PATH

# image sizes as HEIGHTxWIDTH arguments (both multiples of 32), e.g. ./build_trt.sh 1856x2880; default 512x640 480x640.
# The exported ONNX models take any such size, the engines are built for one size each.
SIZES=${@:-512x640 480x640}

# FP32 engines: with TensorRT 10 on RTX 50xx the --fp16 engines return wrong depths (tens of metres off)
for SIZE in $SIZES; do
  H=${SIZE%x*}
  W=${SIZE#*x}
  echo ">>> Building TensorRT engine: ${H}x${W}"
  $TRT_BIN \
    --onnx=$ONNX \
    --saveEngine=$ENGINE_DIR/spnet_${H}_${W}.engine \
    --optShapes=rgb:1x3x${H}x${W},depth:1x1x${H}x${W},mask:1x1x${H}x${W}
done

echo ">>> TensorRT engine build finished."
