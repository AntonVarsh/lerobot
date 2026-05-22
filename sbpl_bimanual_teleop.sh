#!/bin/bash

trap 'kill $(jobs -p) 2>/dev/null; wait; exit' INT TERM

lerobot-teleoperate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM1 \
    --robot.id=sbpl_l_follower \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM2 \
    --teleop.id=sbpl_l_leader &

lerobot-teleoperate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=sbpl_r_follower \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM3 \
    --teleop.id=sbpl_r_leader &

wait
