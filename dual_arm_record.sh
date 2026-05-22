#!/bin/bash

DISPLAY_DATA=${1:-false}

lerobot-record \                                                                                        ─╯
    --robot.type=bi_so_follower \
    --robot.id=sbpl_bimanual_follower \
    --robot.left_arm_config.port=/dev/ttyACM1 \
    --robot.left_arm_config.cameras='{ wrist: {type: opencv, index_or_path: /dev/video8, width: 640, height:
  480, fps: 30}, top: {"type": "opencv", "index_or_path": /dev/video14, "width": 640, "height": 480, "fps": 30}}' \
    --robot.right_arm_config.port=/dev/ttyACM0 \
    --robot.right_arm_config.cameras='{ wrist: {type: opencv, index_or_path: /dev/video6, width: 640, height:
  480, fps: 30}}' \
    --teleop.type=bi_so_leader \
    --teleop.id=sbpl_bimanual_leader \
    --teleop.left_arm_config.port=/dev/ttyACM2 \
    --teleop.right_arm_config.port=/dev/ttyACM3 \
    --display_data=true \
    --dataset.num_episodes=5 \
    --dataset.single_task="Grab and handover the red cube to the other arm" \
    --dataset.repo_id=${HF_USER}/bimanual-so-handover-cube \
    --dataset.streaming_encoding=true \
    --dataset.encoder_threads=2

