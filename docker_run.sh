#!/bin/bash

target="${1}"

case "${target}" in
"eval")
    volume="output:/output"
    ;;
"dev")
    volume="output:/output aichallenge:/aichallenge remote:/remote vehicle:/vehicle"
    ;;
*)
    echo "invalid argument (use 'dev' or 'eval')"
    exit 1
    ;;
esac

if command -v nvidia-smi &>/dev/null && [[ -e /dev/nvidia0 ]]; then
    opts="--nvidia"
    echo "[INFO] NVIDIA GPU detected → enabling --nvidia"
else
    opts=""
    echo "[INFO] No NVIDIA GPU detected → running on CPU"
fi

mkdir -p output

# shellcheck disable=SC2086
rocker "${opts}" --x11 --devices /dev/dri --user --net host --privileged --name "aichallenge-2024-$(date "+%Y-%m-%d-%H-%M-%S")" --volume ${volume} -- "aichallenge-2024-${target}-${USER}"
