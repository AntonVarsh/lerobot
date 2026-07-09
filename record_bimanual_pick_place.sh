#!/bin/bash

TASK=${1:?"Usage: $0 <task_description> <repo_name> [num_episodes] [resume] [display_data]"}
REPO_NAME=${2:?"Usage: $0 <task_description> <repo_name> [num_episodes] [resume] [display_data]"}
NUM_EPISODES=${3:-1}
RESUME=${4:-false}
DISPLAY_DATA=${5:-true}

if [[ -z "${HF_USER}" ]]; then
    HF_BIN=$(which hf 2>/dev/null || echo "/home/itamar/miniconda3/envs/lerobot/bin/hf")
    HF_USER=$(NO_COLOR=1 "${HF_BIN}" auth whoami 2>/dev/null | grep 'user:' | awk '{print $NF}')
    if [[ -z "${HF_USER}" ]]; then
        echo "Error: HF_USER is not set and could not be detected. Run: export HF_USER=<your_huggingface_username>"
        exit 1
    fi
    echo "Detected HF_USER: ${HF_USER}"
fi

lerobot-record \
    --robot.type=bi_so_follower \
    --robot.id=home_bimanual_follower \
    --robot.left_arm_config.port=/dev/ttyACM1 \
    --robot.right_arm_config.port=/dev/ttyACM0 \
    --robot.left_arm_config.cameras='{ wrist: {type: opencv, index_or_path: /dev/video6, width: 640, height: 480, fps: 30}, top: {"type": "opencv", "index_or_path": /dev/video8, "width": 640, "height": 480, "fps": 30}}' \
    --robot.right_arm_config.cameras='{ wrist: {type: opencv, index_or_path: /dev/video4, width: 640, height: 480, fps: 30}}' \
    --teleop.type=bi_so_leader \
    --teleop.id=home_bimanual_leader \
    --teleop.left_arm_config.port=/dev/ttyACM3 \
    --teleop.right_arm_config.port=/dev/ttyACM2 \
    --dataset.num_episodes=${NUM_EPISODES} \
    --dataset.single_task="${TASK}" \
    --dataset.repo_id=${HF_USER}/${REPO_NAME} \
    --dataset.streaming_encoding=true \
    --dataset.encoder_threads=2 \
    --resume=${RESUME} \
    --display_data=${DISPLAY_DATA}
