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

LOG_FILE="output/latest/docker_build.log"
touch $LOG_FILE
echo "A build log is stored at : file://$LOG_FILE"

# shellcheck disable=SC2086
docker build ${opts} --progress=plain --target "${target}" -t "aichallenge-2024-${target}-${USER}" .  2>&1 | tee "$LOG_FILE"
echo "========================================================"
echo "This log is in : file://$LOG_FILE"
echo "========================================================"
