#!/bin/bash
# Builds the gaussian-lic image for the GPU of this machine:
#   docker/build.sh
# Extra arguments go to docker build. Set CUDA_ARCH (e.g. 8.9) to build for another GPU, GAUSSIAN_LIC_IMAGE to name it.
set -e

# added to the docker group, but not logged in again since: go through the group until then
if [ ! -w /var/run/docker.sock ] && id -nG "$USER" | grep -qw docker && ! id -nG | grep -qw docker; then
  exec sg docker -c "$(printf '%q ' "$0" "$@")"
fi

arch=${CUDA_ARCH:-$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader 2>/dev/null | head -n 1)}
if [ -z "$arch" ]; then
  echo "No NVIDIA GPU found with nvidia-smi; give its compute capability, e.g. CUDA_ARCH=8.9 $0" >&2
  exit 1
fi
echo ">>> Building ${GAUSSIAN_LIC_IMAGE:-gaussian-lic} for GPU compute capability $arch"
cd "$(dirname "$0")/.."
exec docker build -t "${GAUSSIAN_LIC_IMAGE:-gaussian-lic}" --build-arg CUDA_ARCH="$arch" -f docker/Dockerfile "$@" .
