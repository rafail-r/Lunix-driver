#!/usr/bin/sh

rmmod ./lunix.ko
insmod ./lunix.ko
./lunix_dev_nodes.sh
./lunix-attach /dev/ttyS0
