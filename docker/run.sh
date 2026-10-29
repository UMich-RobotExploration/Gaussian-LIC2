#!/bin/bash
# Run a command in the gaussian-lic container, or a shell when none is given, e.g.
#   docker/run.sh ros2 launch gaussian_lic payload.launch.xml bag_path:=$HOME/payload_20261021_161730_replayed
#
# Your home folder is mounted at the same path, so bag paths are the same inside and outside the container.
# Results land in ~/gaussian_lic_result: gaussian_lic/ holds the renders, metrics and point_cloud.ply of the latest
# run, cocolic/ the trajectory of every bag (TUM format). The TensorRT engines are kept in ~/.cache/gaussian_lic.
# Override with GAUSSIAN_LIC_DATA (folder to mount), GAUSSIAN_LIC_RESULT, GAUSSIAN_LIC_CACHE, GAUSSIAN_LIC_IMAGE.
set -e

# added to the docker group, but not logged in again since: go through the group until then
if [ ! -w /var/run/docker.sock ] && id -nG "$USER" | grep -qw docker && ! id -nG | grep -qw docker; then
  exec sg docker -c "$(printf '%q ' "$0" "$@")"
fi

IMAGE=${GAUSSIAN_LIC_IMAGE:-gaussian-lic}
DATA=${GAUSSIAN_LIC_DATA:-$HOME}
RESULT=${GAUSSIAN_LIC_RESULT:-$HOME/gaussian_lic_result}
CACHE=${GAUSSIAN_LIC_CACHE:-${XDG_CACHE_HOME:-$HOME/.cache}/gaussian_lic}
mkdir -p "$RESULT" "$CACHE"  # created by you; Docker would create them owned by root

args=(--rm --init --gpus all --network host --ipc host --user "$(id -u):$(id -g)"
      -e ROS_DOMAIN_ID -v "$DATA:$DATA" -v "$CACHE:/cache" -v "$RESULT:/results"
      -e GAUSSIAN_LIC_DATA="$DATA" -e GAUSSIAN_LIC_HOST_RESULT="$RESULT")  # where the GUI looks, what it shows
[ -t 0 ] && [ -t 1 ] && args+=(-it)
if [ -n "$DISPLAY" ]; then  # RViz2 on your X server, as you
  args+=(-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro)
  xauth=${XAUTHORITY:-$HOME/.Xauthority}
  [ -f "$xauth" ] && args+=(-e XAUTHORITY="$xauth" -v "$xauth:$xauth:ro")
fi

exec docker run "${args[@]}" "$IMAGE" "$@"
