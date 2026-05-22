#!/bin/bash

DISPLAY_DATA=${1:-false}

lerobot-teleoperate \
    --robot.type=bi_so_follower \
    --robot.id=sbpl_bimanual_follower \
    --robot.left_arm_config.port=/dev/ttyACM1 \
    --robot.left_arm_config.cameras='{ wrist: {type: opencv, index_or_path: /dev/video8, width: 640, height: 480, fps: 30}}' \
    --robot.right_arm_config.port=/dev/ttyACM0 \
    --robot.right_arm_config.cameras='{ wrist: {type: opencv, index_or_path: /dev/video6, width: 640, height: 480, fps: 30}}' \
    --teleop.type=bi_so_leader \
    --teleop.id=sbpl_bimanual_leader \
    --teleop.left_arm_config.port=/dev/ttyACM2 \
    --teleop.right_arm_config.port=/dev/ttyACM3 \
    --display_data=${DISPLAY_DATA}

