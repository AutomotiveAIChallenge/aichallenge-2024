#!/bin/bash

# use this at breaking change ! commit or build fail
sudo rm -r autoware/install/* autoware/build/*
bash build_autoware.sh
