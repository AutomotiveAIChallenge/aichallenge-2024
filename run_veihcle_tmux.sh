#!/bin/bash

bash stop_vehicle_tmux.sh
tmux new-session \; source-file run_script.tmux
