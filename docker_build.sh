#!/bin/bash

target=${1}

case "${target}" in
"eval")
    opts="--no-cache"
    ;;
"dev")
    opts="--no-cache"
    ;;
*)
    echo "invalid argument (use 'dev' or 'eval')"
    exit 1
    ;;
esac

sudo modprobe msr
sudo setfacl -m u:"$(whoami)":r /dev/cpu/*/msr
# shellcheck disable=SC2086
docker build ${opts} --target "${target}" -t "aichallenge-2024-${target}-${USER}" .
