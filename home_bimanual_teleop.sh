#!/bin/bash

lerobot-teleoperate \  
	--robot.type=bi_so_follower \                                               
	--robot.id=home_bimanual_follower \
	--robot.left_arm_config.port=/dev/ttyACM1 \
	--robot.right_arm_config.port=/dev/ttyACM0 \
	--teleop.type=bi_so_leader \
	--teleop.id=home_bimanual_leader \
	--teleop.left_arm_config.port=/dev/ttyACM3 \
	--teleop.right_arm_config.port=/dev/ttyACM2

