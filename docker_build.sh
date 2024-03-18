#!/bin/bash

target=${1}

case "${target}" in
"eval")
    opts="--no-cache"
    ;;
"dev")
    opts=""
    ;;
*)
    echo "invalid argument (use 'dev' or 'eval')"
    exit 1
    ;;
esac

# shellcheck disable=SC2086
docker build ${opts} --target "${target}" -t "aichallenge-source-materials-${target}" .
