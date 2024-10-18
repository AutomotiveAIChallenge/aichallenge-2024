#!/bin/bash

# Kill all running containers
docker kill $(docker ps -q)

# Remove all containers (including stopped ones)
docker rm -f $(docker ps -a -q)

# kill tmux server
tmux kill-server
