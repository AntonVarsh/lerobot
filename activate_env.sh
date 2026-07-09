#!/bin/bash
SETUP=$1

# Inline conda init
__conda_setup="$('/home/itamar/miniconda3/bin/conda' 'shell.bash' 'hook' 2>/dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "/home/itamar/miniconda3/etc/profile.d/conda.sh" ]; then
        . "/home/itamar/miniconda3/etc/profile.d/conda.sh"
    else
        export PATH="/home/itamar/miniconda3/bin:$PATH"
    fi
fi
unset __conda_setup

conda activate lerobot

HF_USER=$(NO_COLOR=1 hf auth whoami | awk '/user:/ {print $2}')
echo $HF_USER

sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM1

if [ "$SETUP" = "home" ]; then
    echo "Follower port is /dev/ttyACM0"
    echo "Leader port is /dev/ttyACM1"
else
    sudo chmod 666 /dev/ttyACM2
    sudo chmod 666 /dev/ttyACM3
fi
