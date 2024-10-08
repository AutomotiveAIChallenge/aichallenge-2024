#!/bin/bash

# kill tmux server
tmux kill-server
# kill all docker
docker rm -f "$(docker ps -a -q)"
