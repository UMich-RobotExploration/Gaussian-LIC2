#!/bin/bash
# Start the Gaussian-LIC GUI in the container and open it in your default browser once it is up, e.g.
#   docker/gui.sh
#   docker/gui.sh bag_folder:=$HOME/bags port:=8081
# Arguments go to gui.launch.xml. Stop it with Ctrl-C.

port=8080
for arg; do [[ $arg == port:=* ]] && port=${arg#port:=}; done

if [ -n "$DISPLAY$WAYLAND_DISPLAY" ] && command -v xdg-open > /dev/null; then
  (for _ in $(seq 600); do  # the first start builds the TensorRT engines first
     kill -0 $$ 2> /dev/null || exit  # stopped before the page came up
     if curl -fs -o /dev/null "http://localhost:$port"; then
       xdg-open "http://localhost:$port" > /dev/null 2>&1
       exit
     fi
     sleep 1
   done) &
  opener=$!
fi

"$(dirname "$0")/run.sh" ros2 launch gaussian_lic gui.launch.xml open_browser:=false "$@"
status=$?
[ -n "$opener" ] && kill "$opener" 2> /dev/null
exit $status
