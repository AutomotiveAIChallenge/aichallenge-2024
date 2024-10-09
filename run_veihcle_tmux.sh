#!/bin/bash

bash stop_vehicle_tmux.sh
mux new-session \; source-file run_script.tmux
