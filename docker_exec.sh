#!/bin/bash

# Display title
echo "======================================"
echo "  Docker Container Exec Tool"
echo "======================================"
echo ""

# Check running containers
RUNNING_CONTAINERS=$(docker ps -q)
if [ -z "$RUNNING_CONTAINERS" ]; then
    echo "No running Docker containers found."
    exit 1
fi

# Display running container list (numbered)
echo "Running containers:"
echo "--------------------------------------"
docker ps --format "{{.ID}}\t{{.Names}}" | nl -w2 -s') '
echo "--------------------------------------"
echo ""

# Ask user to select a container by number
echo "Enter the number of the container you want to execute a command in:"
read SELECTED_CONTAINER
docker exec -it $SELECTED_CONTAINER bash
