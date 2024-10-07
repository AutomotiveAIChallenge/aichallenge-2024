#!/bin/bash

target=${1}
device=${2}

case "${target}" in
"eval")
    volume="output:/output"
    ;;
"dev")
    volume="output:/output aichallenge:/aichallenge"
    ;;
*)
    echo "invalid argument (use 'dev' or 'eval')"
    exit 1
    ;;
esac

case "${device}" in
"cpu")
    opts=""
    ;;
"gpu")
    opts="--nvidia"
    ;;
*)
    echo "invalid argument (use 'gpu' or 'cpu')"
    exit 1
    ;;
esac

mkdir -p output

# shellcheck disable=SC2086
rocker ${opts} --x11 --devices /dev/dri --user --net host --privileged --name aichallenge-2024-$(date "+%Y-%m-%d-%H-%M-%S") --volume ${volume} -- "aichallenge-2024-${target}-${USER}"
