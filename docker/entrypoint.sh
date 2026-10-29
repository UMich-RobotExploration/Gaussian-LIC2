#!/bin/bash
# Container entrypoint: sets up the ROS 2 environment, builds the SPNet TensorRT engines for every image size in
# the Gaussian-LIC configs that has none yet (once, they are kept in /cache), then runs the command.
WS=${GAUSSIAN_LIC_WS:-/opt/gaussian_lic_ws}
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

set -e
ENGINE_DIR=${ENGINE_DIR:-/cache/engines}
RESULTS_DIR=${RESULTS_DIR:-/results}  # the install tree links its result folders here
mkdir -p "$HOME" "$ENGINE_DIR" "$RESULTS_DIR/gaussian_lic" "$RESULTS_DIR/cocolic"

for config in "$WS"/install/gaussian_lic/share/gaussian_lic/config/*.yaml; do
  awk '/^height:/ {h = $2} /^width:/ {w = $2} END {if (h && w) print h "x" w}' "$config"
done | sort -u | while read -r size; do
  [ -f "$ENGINE_DIR/spnet_${size/x/_}.engine" ] && continue
  echo ">>> Building the depth completion engine for ${size} images (once, takes a minute or two)"
  if ! ONNX=${ONNX:-/opt/spnet/spnet.onnx} ENGINE_DIR=$ENGINE_DIR TENSORRT_ROOT=${TENSORRT_ROOT:-/opt/TensorRT-10.16.1.11} \
       CUDA_ROOT=${CUDA_ROOT:-/usr/local/cuda} "${BUILD_TRT:-/opt/spnet/build_trt.sh}" "$size" > "$ENGINE_DIR/build_${size}.log" 2>&1; then
    tail -n 20 "$ENGINE_DIR/build_${size}.log"
    echo ">>> Engine build failed, see $ENGINE_DIR/build_${size}.log. Is the GPU available (docker run --gpus all)?"
    rm -f "$ENGINE_DIR/spnet_${size/x/_}.engine"
    exit 1
  fi
done

exec "$@"
