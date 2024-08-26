#!/bin/sh
for i in $(seq 2 100000)
do
    kill ${i}
done
