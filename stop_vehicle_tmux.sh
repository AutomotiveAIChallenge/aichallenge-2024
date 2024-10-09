#!/bin/bash

# Stop all running containers
docker stop $(docker ps -q)

# Kill all running containers (if needed)
docker kill $(docker ps -q)

# Remove all containers (including stopped ones)
docker rm -f $(docker ps -a -q)

# kill tmux server
tmux kill-server
