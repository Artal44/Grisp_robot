#!/bin/bash
rebar3 compile
sudo rm -rf /media/nicolas/GRISP/sensor/lib/sensor-0.1.0/src/*

cp ./_build/default/lib/sensor/src/* /media/nicolas/GRISP/sensor/lib/sensor-0.1.0/src/
cp ./_build/default/lib/sensor/ebin/* /media/nicolas/GRISP/sensor/lib/sensor-0.1.0/ebin
umount /media/nicolas/GRISP