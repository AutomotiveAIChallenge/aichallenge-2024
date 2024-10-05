#!/bin/bash
rocker --x11 --devices /dev/dri --volume /dev/vcu --user --user-preserve-groups dialout --net host --privileged --name racing_kart_interface ghcr.io/automotiveaichallenge/racing_kart_interface "$@"
