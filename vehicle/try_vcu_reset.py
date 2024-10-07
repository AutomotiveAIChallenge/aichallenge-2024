#!/usr/bin/env python3
# This script send Ctrl + C several times, then send Ctrl + D.

import argparse
import serial
import time

parser = argparse.ArgumentParser()
parser.add_argument("--device", default="/dev/vcu/usb")
args = parser.parse_args()

device = serial.Serial(args.device, 115200, timeout=None)
for _ in range(10):
    device.write(b"\x03")
    time.sleep(0.1)
device.write(b"\x04")
device.close()
