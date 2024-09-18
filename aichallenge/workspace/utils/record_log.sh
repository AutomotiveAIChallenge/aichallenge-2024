#!/usr/bin/bash

NODES_TO_RECORD=("localization/ekf_localizer" "simple_pure_pursuit_node" "racing_kart_driver")

source $HOME/aichallenge-2024/aichallenge/workspace/install/setup.bash
source $HOME/racing_kart_interface/install/setup.bash

TIMESTAMP=$(date +%Y-%m-%d-%H-%M-%S)

RECORD_DIR="$HOME/rosbag/$(date +%m%d)/$TIMESTAMP"

mkdir -p "$RECORD_DIR"

for node in "${NODES_TO_RECORD[@]}"; do
    ros2 param dump "/$node" >>"$RECORD_DIR/node_param.yaml"
    echo "Recorded parameters for /$node"
done

ros2 bag record -a -o "$RECORD_DIR/$TIMESTAMP"

echo "Rosbag recording finished. Compressing data..."

mv "$RECORD_DIR/$TIMESTAMP"/* "$RECORD_DIR"
rmdir "$RECORD_DIR/$TIMESTAMP"

cd "$RECORD_DIR" || exit

zip -r "$RECORD_DIR.zip" "./"

echo "Compression completed. ZIP file created at $RECORD_DIR.zip"
