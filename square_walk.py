#!/usr/bin/env python3
"""walk spot in a square: forward (side_m) -> turn left 90 degrees -> repeat 4 times"""

import argparse
import math
import time

import bosdyn.client
import bosdyn.client.util
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_command import (
    RobotCommandClient,
    RobotCommandBuilder,
    block_for_trajectory_cmd,
)


def move(cmd_client, robot, x, y, yaw, seconds):
    """Move relative to Spot's current body frame, then wait until done."""
    snapshot = robot.get_frame_tree_snapshot()

    cmd = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(
        goal_x_rt_body=x,
        goal_y_rt_body=y,
        goal_heading_rt_body=yaw,
        frame_tree_snapshot=snapshot,
    )

    cmd_id = cmd_client.robot_command(cmd, end_time_secs=time.time() + seconds)
    block_for_trajectory_cmd(cmd_client, cmd_id, timeout_sec=seconds + 10)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("robot_ip", help="Spot IP")
    p.add_argument("--side", type=float, default=1.0, help="Side length (meters)")
    p.add_argument("--lin_speed", type=float, default=0.35, help="Forward speed guess (m/s)")
    p.add_argument("--yaw_rate", type=float, default=0.6, help="Turn rate guess (rad/s)")
    p.add_argument("--pause", type=float, default=0.4, help="Pause between moves (sec)")
    args = p.parse_args()

    bosdyn.client.util.setup_logging()

    sdk = bosdyn.client.create_standard_sdk("SquareWalk")
    robot = sdk.create_robot(args.robot_ip)


    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()

    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    cmd_client = robot.ensure_client(RobotCommandClient.default_service_name)

    forward_time = max(0.8, args.side / max(0.05, args.lin_speed))
    turn_time = max(0.8, (math.pi / 2) / max(0.05, args.yaw_rate))

    with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        # trying to get lease to just me
        robot.logger.info("Powering on...")
        robot.power_on(timeout_sec=20)
        assert robot.is_powered_on(), "Robot power on failed."

        # stand first
        cmd_client.robot_command(RobotCommandBuilder.synchro_stand_command())
        time.sleep(2)

        for i in range(4):
            print(f"Leg {i+1}/4: forward")
            move(cmd_client, robot, args.side, 0.0, 0.0, forward_time)
            time.sleep(args.pause)

            print("Turn left 90")
            move(cmd_client, robot, 0.0, 0.0, math.pi / 2, turn_time)
            time.sleep(args.pause)


        cmd_client.robot_command(RobotCommandBuilder.synchro_sit_command())
        time.sleep(2)

    print("Done.")


if __name__ == "__main__":
    main()

# to run this script:
# activate environment:
#       source ~/spot/spot310/bin/activate
# set spot login credentials:
#       export BOSDYN_CLIENT_USERNAME=
#       export BOSDYN_CLIENT_PASSWORD=
# run the sq walk script
#       python3 spot-sdk/python/examples/square_walk/square_walk.py 192.168.80.3 --side 1.0
