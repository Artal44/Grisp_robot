#!/bin/bash
rebar3 compile
sudo rm -rf /media/arthur/GRISP/balancing_robot/lib/balancing_robot-0.1.0/src/*

cp ./_build/default/lib/balancing_robot/src/* /media/arthur/GRISP/balancing_robot/lib/balancing_robot-0.1.0/src/
cp ./_build/default/lib/balancing_robot/ebin/* /media/arthur/GRISP/balancing_robot/lib/balancing_robot-0.1.0/ebin
umount /media/arthur/GRISP