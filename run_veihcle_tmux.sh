#!/bin/bash

bash stop_vehicle_tmux.sh
gnome-terminal --maximize --zoom 0.7 -- tmux new-session \; source-file run_script.tmux
