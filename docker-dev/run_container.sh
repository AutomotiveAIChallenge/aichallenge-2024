#!/bin/bash
case "${1:-gpu}" in
  "cpu")
    args="--device /dev/dri" ;;
  "gpu")
    args="--nvidia" ;;
  *)
    echo "invalid argument (use 'gpu' or 'cpu')"; exit 1;;
esac

rocker ${args} --rmw cyclonedds --x11 --user --net host --privileged --volume ../aichallenge:/aichallenge -- aichallenge-yyyymm-dev
