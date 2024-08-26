#!/bin/bash

# Autowareのセットアップスクリプトをソースする
source /autoware/install/setup.bash

# ワークスペースディレクトリに移動
cd /aichallenge/workspace

sudo apt update

# rosdepの更新
rosdep update

# 依存関係のインストール
rosdep install -y -r -i --from-paths src --ignore-src --rosdistro $ROS_DISTRO
