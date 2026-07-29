#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import time
from functools import cached_property

from lerobot.cameras import make_cameras_from_configs
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_so_follower import SO107FollowerRobotConfig, SOFollowerRobotConfig

logger = logging.getLogger(__name__)


class SOFollower(Robot):
    """
    Generic SO follower base implementing common functionality for SO-100/101/10X.
    Designed to be subclassed with a per-hardware-model `config_class` and `name`.
    """

    config_class = SOFollowerRobotConfig
    name = "so_follower"

    # Joints that can spin continuously (no mechanical hard stop), so calibration assumes
    # a full 0-4095 encoder turn instead of recording a measured range of motion.
    full_turn_motors: tuple[str, ...] = ("wrist_roll",)

    def _motors(self, norm_mode_body: MotorNormMode) -> dict[str, Motor]:
        return {
            "shoulder_pan": Motor(1, "sts3215", norm_mode_body),
            "shoulder_lift": Motor(2, "sts3215", norm_mode_body),
            "elbow_flex": Motor(3, "sts3215", norm_mode_body),
            "wrist_flex": Motor(4, "sts3215", norm_mode_body),
            "wrist_roll": Motor(5, "sts3215", norm_mode_body),
            "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
        }

    def __init__(self, config: SOFollowerRobotConfig):
        super().__init__(config)
        self.config = config
        # choose normalization mode depending on config if available
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        self.bus = FeetechMotorsBus(
            port=self.config.port,
            motors=self._motors(norm_mode_body),
            calibration=self.calibration,
        )
        self.cameras = make_cameras_from_configs(config.cameras)

        # Previous continuous forearm-roll target received from the leader.
        # Used to convert an absolute leader angle into a small relative step.
        self._last_forearm_roll_target: float | None = None

        # True after startup alignment and switching forearm_roll to STEP mode.
        self._forearm_roll_synchronized: bool = False


    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        """
        We assume that at connection time, arm is in a rest position,
        and torque can be safely disabled to run calibration.
        """

        self.bus.connect()

        # Reset forearm-roll step tracking after reconnecting.
        self._last_forearm_roll_target = None
        self._forearm_roll_synchronized = False

        if not self.is_calibrated and calibrate:
            logger.info(
                "Mismatch between calibration values in the motor and the calibration file or no calibration file found"
            )
            self.calibrate()

        for cam in self.cameras.values():
            cam.connect()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        if self.calibration:
            # Calibration file exists, ask user whether to use it or run new calibration
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, or type 'c' and press ENTER to run calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Writing calibration file associated with the id {self.id} to the motors")
                self.bus.write_calibration(self.calibration)
                return

        logger.info(f"\nRunning calibration of {self}")
        self.bus.disable_torque()
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        input(f"Move {self} to the middle of its range of motion and press ENTER....")
        homing_offsets = self.bus.set_half_turn_homings()

        # Attempt to call record_ranges_of_motion with a reduced motor set when appropriate.
        full_turn_motors = [motor for motor in self.full_turn_motors if motor in self.bus.motors]
        unknown_range_motors = [motor for motor in self.bus.motors if motor not in full_turn_motors]
        print(
            f"Move all joints except {full_turn_motors} sequentially through their "
            "entire ranges of motion.\nRecording positions. Press ENTER to stop..."
        )
        range_mins, range_maxes = self.bus.record_ranges_of_motion(unknown_range_motors)
        for motor in full_turn_motors:
            range_mins[motor] = 0
            range_maxes[motor] = 4095

        self.calibration = {}
        for motor, m in self.bus.motors.items():
            self.calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=homing_offsets[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )

        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        print("Calibration saved to", self.calibration_fpath)


    def configure(self) -> None:
        with self.bus.torque_disabled():
            self.bus.configure_motors()

            for motor in self.bus.motors:
                if motor == "forearm_roll":
                    self.bus.write(
                        "Min_Position_Limit",
                        motor,
                        0,
                        normalize=False,
                    )
                    self.bus.write(
                        "Max_Position_Limit",
                        motor,
                        4095,
                        normalize=False,
                    )
                    self.bus.write(
                        "Operating_Mode",
                        motor,
                        OperatingMode.POSITION.value,
                    )
                else:
                    self.bus.write(
                        "Operating_Mode",
                        motor,
                        OperatingMode.POSITION.value,
                    )

                self.bus.write("P_Coefficient", motor, 16)
                self.bus.write("I_Coefficient", motor, 0)
                self.bus.write("D_Coefficient", motor, 32)

                if motor == "gripper":
                    self.bus.write("Max_Torque_Limit", motor, 500)
                    self.bus.write("Protection_Current", motor, 250)
                    self.bus.write("Overload_Torque", motor, 25)

    def _switch_forearm_roll_to_step(self) -> None:
        """Switch forearm_roll from absolute POSITION mode to relative STEP mode."""

        with self.bus.torque_disabled():
            # Disable the one-turn absolute-position limits.
            self.bus.write(
                "Min_Position_Limit",
                "forearm_roll",
                0,
                normalize=False,
            )
            self.bus.write(
                "Max_Position_Limit",
                "forearm_roll",
                0,
                normalize=False,
            )

            # Interpret future Goal_Position writes as relative movements.
            self.bus.write(
                "Operating_Mode",
                "forearm_roll",
                OperatingMode.STEP.value,
            )

            # Ensure no nonzero command remains in Goal_Position after changing
            # the meaning of the register from absolute position to relative step.
            self.bus.write(
                "Goal_Position",
                "forearm_roll",
                0,
                normalize=False,
            )


    def setup_motors(self) -> None:
        motor_names = list(self.bus.motors)
        done = set()
        while True:
            print("\nMotors:")
            for i, motor in enumerate(motor_names, start=1):
                print(f"  {i}. {motor}" + (" (done)" if motor in done else ""))
            choice = input("Enter a motor number to flash, or press ENTER to finish: ").strip()
            if not choice:
                break
            if not choice.isdigit() or not (1 <= int(choice) <= len(motor_names)):
                print(f"Invalid choice: {choice!r}")
                continue
            motor = motor_names[int(choice) - 1]
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.bus.setup_motor(motor)
            done.add(motor)
            print(f"'{motor}' motor id set to {self.bus.motors[motor].id}")

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        # Read arm position
        start = time.perf_counter()
        obs_dict = self.bus.sync_read("Present_Position")
        obs_dict = {f"{motor}.pos": val for motor, val in obs_dict.items()}
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.read_latest()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict


    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        """Command the arm, using relative step control for forearm_roll."""

        goal_pos = {
            key.removesuffix(".pos"): val
            for key, val in action.items()
            if key.endswith(".pos")
        }

        # Remove forearm_roll from the normal absolute-position write.
        forearm_roll_target = goal_pos.pop("forearm_roll", None)

        # Normal position control for every other joint.
        if self.config.max_relative_target is not None and goal_pos:
            present_pos = self.bus.sync_read("Present_Position")
            goal_present_pos = {
                key: (g_pos, present_pos[key])
                for key, g_pos in goal_pos.items()
            }
            goal_pos = ensure_safe_goal_position(
                goal_present_pos,
                self.config.max_relative_target,
            )

        if goal_pos:
            self.bus.sync_write("Goal_Position", goal_pos)

        # Align forearm_roll in POSITION mode, then track it in STEP mode.
        if forearm_roll_target is not None:
            forearm_roll_target = float(forearm_roll_target)

            if not self._forearm_roll_synchronized:
                present_pos = self.bus.sync_read("Present_Position")
                follower_angle = float(present_pos["forearm_roll"])

                # Preserve the full reported angle; do not reduce it modulo 360.
                position_target = forearm_roll_target
                error_degrees = position_target - follower_angle

                sync_tolerance_degrees = 2.0

                if abs(error_degrees) <= sync_tolerance_degrees:

                    self._switch_forearm_roll_to_step()

                    # Establish the leader baseline so the first STEP command is zero.
                    self._last_forearm_roll_target = forearm_roll_target
                    self._forearm_roll_synchronized = True

                    print(
                        "[forearm_roll position sync] "
                        "Alignment complete; switched to STEP mode."
                    )
                else:
                    print(
                        f"[forearm_roll position sync] "
                        f"commanding absolute target={position_target:.1f}°"
                    )

                    self.bus.sync_write(
                        "Goal_Position",
                        {"forearm_roll": position_target},
                    )

            else:
                # Once in STEP mode, copy only the leader's movement since the
                # preceding control-loop frame.
                delta_degrees = (
                    forearm_roll_target
                    - self._last_forearm_roll_target
                )

                delta_raw = round(delta_degrees * 4096.0 / 360.0)


                # Prevent one bad reading from producing a large sudden movement.
                max_step_raw = 100
                delta_raw = max(
                    -max_step_raw,
                    min(max_step_raw, delta_raw),
                )

                if delta_raw != 0:
                    self.bus.sync_write(
                        "Goal_Position",
                        {"forearm_roll": delta_raw},
                        normalize=False,
                    )

                self._last_forearm_roll_target = forearm_roll_target

        returned_action = {
            f"{motor}.pos": val
            for motor, val in goal_pos.items()
        }

        if forearm_roll_target is not None:
            returned_action["forearm_roll.pos"] = forearm_roll_target

        return returned_action

    @check_if_not_connected
    def disconnect(self):
        self.bus.disconnect(self.config.disable_torque_on_disconnect)
        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")


class SO107Follower(SOFollower):
    """
    SO-107 follower: SO-101 plus a `forearm_roll` joint inserted between `elbow_flex` and
    `wrist_flex`. Like `wrist_roll`, `forearm_roll` spins continuously with no hard stop.

    Motor ids are sequential in physical chain order (1-7), matching the SO-101 convention.
    Since `forearm_roll` is inserted at id 4, `wrist_flex`/`wrist_roll`/`gripper` shift up by
    one from their SO-101 ids -- re-run `lerobot-setup-motors` for those three (and the new
    motor) after adding the joint; `shoulder_pan`/`shoulder_lift`/`elbow_flex` are unaffected.
    """

    config_class = SO107FollowerRobotConfig
    name = "so107_follower"
    full_turn_motors = ("wrist_roll", "forearm_roll")

    def _motors(self, norm_mode_body: MotorNormMode) -> dict[str, Motor]:
        return {
            "shoulder_pan": Motor(1, "sts3215", norm_mode_body),
            "shoulder_lift": Motor(2, "sts3215", norm_mode_body),
            "elbow_flex": Motor(3, "sts3215", norm_mode_body),
            "forearm_roll": Motor(4, "sts3215", norm_mode_body),
            "wrist_flex": Motor(5, "sts3215", norm_mode_body),
            "wrist_roll": Motor(6, "sts3215", norm_mode_body),
            "gripper": Motor(7, "sts3215", MotorNormMode.RANGE_0_100),
        }


SO100Follower = SOFollower
SO101Follower = SOFollower
