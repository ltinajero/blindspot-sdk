#!/usr/bin/env python3
"""
Press once -> start square walk
Press again (while running) -> STOP + SIT
After sitting (either completed or interrupted) -> reset and wait for next start
"""

import argparse
import math
import time
import threading

import serial  # pip install pyserial

import bosdyn.client
import bosdyn.client.util
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_command import (
    RobotCommandClient,
    RobotCommandBuilder,
# block_for_trajectory_cmd, - dont use this since we want to interrupt the trajectory command if the button is pressed
)

from bosdyn.client.robot_command import blocking_sit
from bosdyn.api import basic_command_pb2

def request_stop_and_sit(cmd_client: RobotCommandClient):
    """Send stop command, then sit."""
    cmd_client.robot_command(RobotCommandBuilder.stop_command())
    time.sleep(0.2)
    blocking_sit(cmd_client)
        
def move_with_interrupt(cmd_client, robot, x, y, yaw, seconds, stop_event: threading.Event):
    """
    Trajectory in body frame with polling so we can interrupt.
    """
    snapshot = robot.get_frame_tree_snapshot()

    cmd = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(
        goal_x_rt_body=x,
        goal_y_rt_body=y,
        goal_heading_rt_body=yaw,
        frame_tree_snapshot=snapshot,
    )
    cmd_id = cmd_client.robot_command(cmd, end_time_secs=time.time() + seconds)

    #start of the interrupt part
    deadline = time.time() + seconds + 10.0
    while time.time() < deadline:
        if stop_event.is_set():
            request_stop_and_sit(cmd_client)
            raise KeyboardInterrupt("Stop requested")

        fb = cmd_client.robot_command_feedback(cmd_id)
        mob = fb.feedback.synchronized_feedback.mobility_command_feedback
        traj_fb = mob.se2_trajectory_feedback

        # Common completion condition in many SDK versions:
        done = {basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_STOPPED}
        if hasattr(basic_command_pb2.SE2TrajectoryCommand.Feedback, "STATUS_AT_GOAL"):
            done.add(basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_AT_GOAL)

        if traj_fb.status in done:
            return

        time.sleep(0.05)

    cmd_client.robot_command(RobotCommandBuilder.stop_command())
    raise TimeoutError("Trajectory timed out")


class ButtonToggle:
    """
    Serial button reader that produces 'press events' (edge-triggered).
    We maintain a press counter per run-cycle and reset it after Spot sits.
    """
    def __init__(self, port: str, baud: int, threshold: int, debounce_sec: float):
        self.port = port
        self.baud = baud
        self.threshold = threshold
        self.debounce_sec = debounce_sec

        self._presses = 0
        self._armed = True  # when True, we count presses
        self._lock = threading.Lock()

        self.start_event = threading.Event()
        self.stop_event = threading.Event()

    def reset_cycle(self):
        """Call after Spot sits to allow a fresh start/stop cycle."""
        with self._lock:
            self._presses = 0
            self._armed = True
            self.start_event.clear()
            self.stop_event.clear()
        print(">>> Reset: waiting for START press.")

    def _handle_press(self):
        """Internal: handle a button press event."""
        with self._lock:
            if not self._armed:
                return
            self._presses += 1
            if self._presses == 1:
                print(">>> Button press #1: START")
                self.start_event.set()
            elif self._presses == 2:
                print(">>> Button press #2: STOP")
                self.stop_event.set()
            else:
                # ignore extra presses within same cycle
                pass

    def run_forever(self):
        """Run in a separate thread to monitor the button."""
        ser = serial.Serial(self.port, self.baud, timeout=1)
        ser.reset_input_buffer()
        print(f">>> Listening on {self.port} @ {self.baud}. threshold={self.threshold}")

        last_pressed_state = False
        last_press_time = 0.0

        while True:
            if ser.in_waiting <= 0:
                time.sleep(0.01)
                continue

            raw = ser.readline().decode("utf-8", errors="ignore").strip()
            if not raw:
                continue

            try:
                value = int(raw)
            except ValueError:
                continue

            pressed_now = (value >= self.threshold)

            # Rising edge: not pressed -> pressed
            if pressed_now and not last_pressed_state:
                now = time.time()
                if now - last_press_time >= self.debounce_sec:
                    last_press_time = now
                    self._handle_press()

            last_pressed_state = pressed_now

def run_square_once(robot, cmd_client, side, lin_speed, yaw_rate, pause, stop_event):
    """Run one square walk cycle. Can be called again after sitting to allow restart."""
    forward_time = max(0.8, side / max(0.05, lin_speed))
    turn_time = max(0.8, (math.pi / 2) / max(0.05, yaw_rate))

    for i in range(4):
        print(f"Leg {i+1}/4: forward")
        move_with_interrupt(cmd_client, robot, side, 0.0, 0.0, forward_time, stop_event)
        time.sleep(pause)

        print("Turn left 90")
        move_with_interrupt(cmd_client, robot, 0.0, 0.0, math.pi / 2, turn_time, stop_event)
        time.sleep(pause)

def main():
    p = argparse.ArgumentParser()
    p.add_argument("robot_ip", help="Spot IP")
    p.add_argument("--side", type=float, default=1.0, help="Side length (meters)")
    p.add_argument("--lin_speed", type=float, default=0.35, help="Forward speed guess (m/s)")
    p.add_argument("--yaw_rate", type=float, default=0.6, help="Turn rate guess (rad/s)")
    p.add_argument("--pause", type=float, default=0.4, help="Pause between moves (sec)")

    # added serial/button args
    p.add_argument("--serial_port", default="/dev/ttyACM0")
    p.add_argument("--serial_baud", type=int, default=9600)
    p.add_argument("--button_threshold", type=int, default=1)
    p.add_argument("--debounce", type=float, default=0.4)

    args = p.parse_args()

    bosdyn.client.util.setup_logging()

    # Start button listener thread once
    button = ButtonToggle(
        port=args.serial_port,
        baud=args.serial_baud,
        threshold=args.button_threshold,
        debounce_sec=args.debounce,
    )
    threading.Thread(target=button.run_forever, daemon=True).start()

    #Connect to Spot once
    sdk = bosdyn.client.create_standard_sdk("SquareWalkLoopToggle")
    robot = sdk.create_robot(args.robot_ip)


    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()

    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    cmd_client = robot.ensure_client(RobotCommandClient.default_service_name)

    button.reset_cycle()

    with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        # trying to get lease to just me
        robot.logger.info("Powering on...")
        robot.power_on(timeout_sec=20)
        assert robot.is_powered_on(), "Robot power on failed."

        # Main loop: wait -> run -> sit -> reset -> repeat
        while True:
            # Ensure we're sitting/idle between runs
            cmd_client.robot_command(RobotCommandBuilder.synchro_sit_command())
            time.sleep(1.5)

            # Wait for start press
            button.start_event.wait()

            # Stand and run one square (unless interrupted)
            cmd_client.robot_command(RobotCommandBuilder.synchro_stand_command())
            time.sleep(2)

            try:
                run_square_once(
                    robot,
                    cmd_client,
                    side=args.side,
                    lin_speed=args.lin_speed,
                    yaw_rate=args.yaw_rate,
                    pause=args.pause,
                    stop_event=button.stop_event,
                )
                print(">>> Square finished normally.")
            except KeyboardInterrupt:
                print(">>> Interrupted by STOP press.")
                # stop+sIt already handled in move_with_interrupt
            except Exception as e:
                print(f">>> Error: {e}\n>>> Stopping and sitting for safety.")
                request_stop_and_sit(cmd_client)
                time.sleep(2)

            # Always end in sit, then reset press counter for next cycle
            cmd_client.robot_command(RobotCommandBuilder.synchro_sit_command())
            time.sleep(2)
            button.reset_cycle()


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
