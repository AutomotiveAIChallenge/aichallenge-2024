#!/bin/bash

# docker setup
AIC_DIR="/home/$USER/aichallenge-2024"
VEHICLE_DIR="/home/$USER/aichallenge-2024/vehicle"

AIC_CD="cd $AIC_DIR"
VEHICLE_CD="cd $VEHICLE_DIR"

AIC_DOCKER_RUN_CMD="bash docker_run.sh dev cpu"
KART_DOCKER_RUN_CMD="bash run_driver.bash"
ZENOH_DOCKER_RUN_CMD="bash run_zenoh.bash"

AIC_WORKSPACE_CD_CMD="cd /aichallenge"
SOURCE_CMD="source install/setup.bash"

# mouse setup
set-option -g default-command "bash --login"
set-option -g default-terminal "screen-256color"
set-option -g mouse on
bind-key -n WheelUpPane if-shell -F -t = "#{mouse_any_flag}" "send-keys -M" "if -Ft= '#{pane_in_mode}' 'send-keys -M' 'select-pane -t=; copy-mode -e; send-keys -M'"
bind-key -n WheelDownPane select-pane -t= \; send-keys -M

set-window-option -g mode-keys vi
bind-key -T copy-mode-vi v send -X begin-selection
bind-key -T copy-mode-vi y send -X copy-pipe-and-cancel "xclip -sel clip -i"


# split-vertical
# 1. 初期のペインで左右に分割
select-pane -t 0          # 最初のペイン (0) を選択
split-window -h -p 50     # 左右に 50% 分割して、右にペイン 1 を作成

# 2. 右側のペインを選択して上下に 3 回分割
select-pane -t 1          # 右側のペイン (1) を選択
split-window -v -p 67     # ペイン 1 を 67% 下で分割し、ペイン 2 を作成
split-window -v -p 50     # ペイン 2 を 50% 下で分割し、ペイン 3 を作成
split-window -v -p 50     # ペイン 3 を 50% 下で分割し、ペイン 4 を作成
# pane 0 is used for aic
select-pane -t 0
send-keys "set -x" C-m
send-keys "$AIC_CD" C-m
send-keys "echo $AIC_WORKSPACE_CD_CMD" C-m
send-keys "echo $SOURCE_CMD" C-m
send-keys "echo run_autoware.bash vehicle" C-m
send-keys "$AIC_DOCKER_RUN_CMD" C-m

# pane 1 is used for aic rosbag record
select-pane -t 1
send-keys "set -x" C-m
send-keys "$AIC_CD" C-m
send-keys "sleep 2" C-m
send-keys "echo $AIC_WORKSPACE_CD_CMD" C-m
send-keys "echo $SOURCE_CMD" C-m
send-keys "$AIC_DOCKER_RUN_CMD" C-m

# pane 2 is used for racing kart docker
select-pane -t 2
send-keys "set -x" C-m
send-keys "$VEHICLE_CD" C-m
send-keys "$KART_DOCKER_RUN_CMD" C-m

# pane 3 is used for zenoh docker
select-pane -t 3
send-keys "set -x" C-m
send-keys "$VEHICLE_CD" C-m
send-keys "sleep 10" C-m
send-keys "$ZENOH_DOCKER_RUN_CMD " C-m

# pane 4 is used for anything
select-pane -t 4
send-keys "set -x" C-m
