import time
import argparse
import torch
import numpy as np
import srmp
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.utils.action_interpolator import ActionInterpolator
from lerobot.utils.robot_utils import precise_sleep

FPS = 30  # Control rate
control_interval = 1 / FPS

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=str, default="/dev/ttyACM0")
    parser.add_argument("--id", type=str, default="home_bimanual_follower_right")
    parser.add_argument("--goal-x", type=float, default=0.0)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--goal-z", type=float, default=0.0)
    parser.add_argument("--goal-rot-x", type=float, default=0.0)
    parser.add_argument("--goal-rot-y", type=float, default=0.0)
    parser.add_argument("--goal-rot-z", type=float, default=0.0)
    args = parser.parse_args()

    # goal pose from input terminal script
    goal_pose = np.array([args.goal_x, args.goal_y, args.goal_z, 
                          args.goal_rot_x, args.goal_rot_y, args.goal_rot_z])

    # find ports using lerobot-find-port
    follower_port = args.port

    # the robot ids are used to load the right calibration files
    follower_id = args.id

    camera_config = {
        # "base_0_rgb": OpenCVCameraConfig(index_or_path=0, width=640, height=480, fps=30),
        # "left_wrist_0_rgb": OpenCVCameraConfig(index_or_path=1, width=640, height=480, fps=30),
        # "right_wrist_0_rgb": OpenCVCameraConfig(index_or_path=2, width=640, height=480, fps=30),
    }

    robot_cfg = SO101FollowerConfig(port=follower_port,
                                    id=follower_id,
                                    cameras=camera_config)
    robot = SO101Follower(robot_cfg)
    robot.connect()

    robot.send_action(
        {
            "shoulder_pan.pos": 0.0,
            "shoulder_lift.pos": np.rad2deg(-np.pi / 2),
            "elbow_flex.pos": np.rad2deg(1.),
            "wrist_flex.pos": np.rad2deg(1.),
            "wrist_roll.pos": -np.rad2deg(np.pi / 2),
            "gripper.pos": 0.0,  # keep gripper closed
        }
    )

    planner: srmp.ViserPlannerInterface = srmp.ViserPlannerInterface(port=8080)
    planner.add_articulation(name="so101", end_effector="gripper_frame_link", 
                             urdf_path="/home/itamar/work/code/lerobot/examples/so100_to_so100_EE/SO101/so101_2.urdf", 
                             srdf_path="/home/itamar/work/code/lerobot/examples/so100_to_so100_EE/SO101/so101.srdf")
    planner.visualize()
    planner.add_robot_controls('so101')
    # while True:
    obs = robot.get_observation()
    current_pos = np.array(
        [
            obs["shoulder_pan.pos"],
            obs["shoulder_lift.pos"],
            obs["elbow_flex.pos"],
            obs["wrist_flex.pos"],
            obs["wrist_roll.pos"],
            obs["gripper.pos"],
        ]
    )
    start_state = np.deg2rad(current_pos)  # convert to radians if necessary
    planner.set_qpos("so101", start_state[:-1])

    start_pose = planner.compute_fk("so101", start_state[:-1])
    print("Start pose:", start_pose)
    goal_pose = srmp.Pose(p=start_pose.p.copy(), q=start_pose.q.copy())  # initialize goal pose to current pose
    goal_pose.p[:3] += np.array([args.goal_x + 0.05, args.goal_y, args.goal_z+0.1])  # add the desired translation to the current position
    # goal_pose[3:] += np.array([args.goal_rot_x, args.goal_rot_y, args.goal_rot_z])  # add the desired rotation to the current orientation
    print("Goal pose:", goal_pose)
    # goal_pose_srmp = srmp.Pose(p=goal_pose[:3], r=goal_pose[3:])
    goal_ee = srmp.GoalConstraint(srmp.GoalType.POSE, [goal_pose])
    planner.make_planner(["so101"], {"planner_id": "wPASE",
                                     "heuristic": "bfs",
                                     "mprim_path": "/home/itamar/work/code/ims_manip/config/manip_5dof_mprim.yaml", 
                                     "weight": "10."})
    # check planning time
    start_time = time.time()
    trajectory: srmp.Trajectory = planner.plan(start_state[:-1], goal_ee)
    
    end_time = time.time()
    print("Planned trajectory:", trajectory)
    print("Planning time:", end_time - start_time)

    # # first visualize:
    # for point in trajectory.positions:
    #     planner.set_qpos("so101", point)
    #     # planner.visualize()
    #     input("Press Enter to continue to the next point...")

    interpolator = ActionInterpolator(multiplier=3)  # 3x smoother

    for point in trajectory.positions:
        action_tensor = torch.tensor(point, dtype=torch.float32)
        interpolator.add(action_tensor)
        
        while not interpolator.needs_new_action():
            interpolated_action = interpolator.get()
            if interpolated_action is None:
                break
            robot.send_action({
                "shoulder_pan.pos": np.degrees(interpolated_action[0].item()),
                "shoulder_lift.pos": np.degrees(interpolated_action[1].item()),
                "elbow_flex.pos": np.degrees(interpolated_action[2].item()),
                "wrist_flex.pos": np.degrees(interpolated_action[3].item()),
                "wrist_roll.pos": np.degrees(interpolated_action[4].item()),
                "gripper.pos": 0.0,
            })
            planner.set_qpos("so101", interpolated_action.numpy())
            precise_sleep(interpolator.get_control_interval(FPS))

    interpolator.reset()
    # revert the trajectory back to the start pose
    for point in reversed(trajectory.positions):
        action_tensor = torch.tensor(point, dtype=torch.float32)
        interpolator.add(action_tensor)
        
        while not interpolator.needs_new_action():
            interpolated_action = interpolator.get()
            if interpolated_action is None:
                break
            robot.send_action({
                "shoulder_pan.pos": np.degrees(interpolated_action[0].item()),
                "shoulder_lift.pos": np.degrees(interpolated_action[1].item()),
                "elbow_flex.pos": np.degrees(interpolated_action[2].item()),
                "wrist_flex.pos": np.degrees(interpolated_action[3].item()),
                "wrist_roll.pos": np.degrees(interpolated_action[4].item()),
                "gripper.pos": 0.0,
            })
            planner.set_qpos("so101", interpolated_action.numpy())
            precise_sleep(interpolator.get_control_interval(FPS))

    # go back to home position
    robot.send_action(
        {
            "shoulder_pan.pos": 0.0,
            "shoulder_lift.pos": np.rad2deg(-np.pi / 2),
            "elbow_flex.pos": np.rad2deg(1.),
            "wrist_flex.pos": np.rad2deg(1.),
            "wrist_roll.pos": -np.rad2deg(np.pi / 2),
            "gripper.pos": 0.0,  # keep gripper closed
        }
    )
    time.sleep(1.0)  # wait for the robot to reach the home position



    

if __name__ == "__main__":    
    main()