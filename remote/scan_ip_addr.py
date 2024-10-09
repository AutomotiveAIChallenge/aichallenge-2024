#!/usr/bin/env python3
import argparse
import subprocess
import sys

macs = {
    "A1": "c0:4b:24:02:a8:9b",
    "A2": "c0:4b:24:02:6e:87",
    "A3": "c0:4b:24:02:a2:97",
    "A4": "a8:43:a4:54:aa:63",
    "A5": "c0:4b:24:c0:fc:fc",
    "A6": "c0:4b:24:02:5d:55",
    "A7": "c0:4b:24:02:9a:41",
    "A8": "c8:8a:d8:1e:b3:81",
}


def exec_arp_scan():
    arpcmd = ["sudo", "arp-scan", "-l", "-g", "--plain", "--quiet"]
    result = subprocess.run(arpcmd, capture_output=True, text=True)
    if result.returncode:
        print("invalid target:", result.stderr, file=sys.stderr)
        sys.exit(result.returncode)
    result = [line.split() for line in result.stdout.split("\n") if line]
    result = {mac: ip for ip, mac in result}
    return result


def show_ip_addr(target):
    mac = macs.get(target)
    if mac is None:
        print("invalid target:", target, file=sys.stderr)
        sys.exit(1)
    result = exec_arp_scan()
    ip = result.get(mac)
    if ip is None:
        print("not found:", target, file=sys.stderr)
        sys.exit(1)
    print(ip)


parser = argparse.ArgumentParser()
parser.add_argument("target")
args = parser.parse_args()
show_ip_addr(args.target)
