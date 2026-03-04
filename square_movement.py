"""Square path movement module.

This module provides functionality to command Spot to walk in a square pattern
using trajectory commands.
"""

import logging
import math
import time

from bosdyn.api import geometry_pb2
from bosdyn.client.frame_helpers import VISION_FRAME_NAME, get_vision_tform_body
from bosdyn.client.robot_command import (
    RobotCommandBuilder, 
    RobotCommandClient, 
    blocking_stand,
    block_for_trajectory_cmd
)
from bosdyn.util import seconds_to_duration


class SquarePathWalker:
    """Commands Spot to walk in a square pattern."""
    
    def __init__(self, robot_command_client, robot_state_client, logger=None):
        """Initialize the square path walker.
        
        Args:
            robot_command_client: RobotCommandClient instance
            robot_state_client: RobotStateClient instance
            logger: Optional logger instance
        """
        self.command_client = robot_command_client
        self.state_client = robot_state_client
        self.logger = logger or logging.getLogger(__name__)
    
    def stand_up(self, timeout_sec=10):
        """Command robot to stand up.
        
        Args:
            timeout_sec: Timeout in seconds
        
        Returns:
            bool: True if successful
        """
        try:
            self.logger.info('Commanding robot to stand...')
            blocking_stand(self.command_client, timeout_sec=timeout_sec)
            self.logger.info('Robot standing.')
            return True
        except Exception as e:
            self.logger.error(f'Stand command failed: {e}')
            return False
    
    def walk_square(self, side_length=1.0, max_velocity=0.5, total_time=20.0):
        """Walk the robot in a square pattern.
        
        This command makes the robot walk in a square with the given side length.
        The robot returns to its starting position.
        
        Args:
            side_length: Length of each side of the square in meters
            max_velocity: Maximum walking velocity in m/s
            total_time: Total time to complete the square in seconds
        
        Returns:
            bool: True if successful
        """
        try:
            self.logger.info(f'Writing square path: side={side_length}m, max_vel={max_velocity}m/s')
            
            # Get current robot state
            robot_state = self.state_client.get_robot_state()
            
            # Get robot's current pose in vision frame
            vision_T_body = get_vision_tform_body(
                robot_state.kinematic_state.transforms_snapshot)
            
            # Define the four corners of a square relative to robot's current position
            # Starting at (0, 0), we go:
            # (0, 0) -> (side, 0) -> (side, side) -> (0, side) -> (0, 0)
            square_points = [
                (0, 0),                      # Starting position
                (side_length, 0),            # Right
                (side_length, side_length),  # Upper right
                (0, side_length),            # Upper left
                (0, 0),                      # Back to start
            ]
            
            # Time for each segment (equal time distribution)
            time_per_segment = total_time / len(square_points)
            
            # Create trajectory points
            trajectory_points = []
            for idx, (x, y) in enumerate(square_points):
                # Transform point from body frame to vision frame
                x_vision, y_vision, _ = vision_T_body.transform_point(x, y, 0)
                
                # Calculate heading (angle) at this point
                if idx < len(square_points) - 1:
                    dx = square_points[idx + 1][0] - square_points[idx][0]
                    dy = square_points[idx + 1][1] - square_points[idx][1]
                    heading = math.atan2(dy, dx)
                else:
                    heading = 0  # Face forward at end
                
                # Create SE2 pose
                pose = geometry_pb2.SE2Pose(
                    position=geometry_pb2.Vec2(x=x_vision, y=y_vision),
                    angle=heading
                )
                
                # Calculate time for this point
                time_at_point = (idx + 1) * time_per_segment
                
                trajectory_points.append((pose, time_at_point))
                self.logger.debug(f'Point {idx}: x={x_vision:.2f}, y={y_vision:.2f}, '
                                f'heading={heading:.2f}, time={time_at_point:.2f}s')
            
            # Build and send trajectory command
            # We'll send waypoints one at a time for better control
            command_id = None
            for pose, time_val in trajectory_points:
                mobility_params = RobotCommandBuilder.mobility_params(
                    max_vel_linear=max_velocity,
                    max_vel_ang=1.0
                )
                
                cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
                    goal_x=pose.position.x,
                    goal_y=pose.position.y,
                    goal_heading=pose.angle,
                    frame_name=VISION_FRAME_NAME,
                    params=mobility_params,
                    body_height=0.0
                )
                
                command_id = self.command_client.robot_command(
                    cmd, 
                    end_time_secs=time.time() + time_val
                )
                self.logger.info(f'Sent waypoint command (ID: {command_id})')
                time.sleep(0.1)  # Small delay between waypoint sends
            
            # Wait for final command to complete
            if command_id:
                self.logger.info('Waiting for robot to reach final waypoint...')
                block_for_trajectory_cmd(self.command_client, command_id, timeout_sec=total_time + 5)
                self.logger.info('Square path complete!')
            
            return True
            
        except Exception as e:
            self.logger.error(f'Square walk failed: {e}')
            return False
    
    def walk_square_trajectory(self, side_length=1.0, max_velocity=0.5, total_time=20.0):
        """Alternative implementation: walk square using full trajectory command.
        
        This approach sends all waypoints in a single trajectory command.
        
        Args:
            side_length: Length of each side of the square in meters
            max_velocity: Maximum walking velocity in m/s
            total_time: Total time to complete the square in seconds
        
        Returns:
            bool: True if successful
        """
        try:
            self.logger.info(f'Writing square trajectory: side={side_length}m')
            
            # Get current robot state and transform
            robot_state = self.state_client.get_robot_state()
            vision_T_body = get_vision_tform_body(
                robot_state.kinematic_state.transforms_snapshot)
            
            # Define square points
            square_points = [
                (0, 0),
                (side_length, 0),
                (side_length, side_length),
                (0, side_length),
                (0, 0),
            ]
            
            time_per_segment = total_time / len(square_points)
            
            # Create all SE2 trajectory points
            from bosdyn.api import trajectory_pb2
            
            traj_points = []
            for idx, (x, y) in enumerate(square_points):
                x_vision, y_vision, _ = vision_T_body.transform_point(x, y, 0)
                
                if idx < len(square_points) - 1:
                    dx = square_points[idx + 1][0] - square_points[idx][0]
                    dy = square_points[idx + 1][1] - square_points[idx][1]
                    heading = math.atan2(dy, dx)
                else:
                    heading = 0
                
                pose = geometry_pb2.SE2Pose(
                    position=geometry_pb2.Vec2(x=x_vision, y=y_vision),
                    angle=heading
                )
                
                time_val = (idx + 1) * time_per_segment
                traj_point = trajectory_pb2.SE2TrajectoryPoint(
                    pose=pose,
                    time_since_reference=seconds_to_duration(time_val)
                )
                traj_points.append(traj_point)
            
            # Create trajectory
            trajectory = trajectory_pb2.SE2Trajectory(points=traj_points)
            
            # Build command with mobility params
            mobility_params = RobotCommandBuilder.mobility_params(
                max_vel_linear=max_velocity,
                max_vel_ang=1.0
            )
            
            cmd = RobotCommandBuilder.synchro_se2_trajectory_command(
                goal_se2=trajectory.points[-1].pose,
                frame_name=VISION_FRAME_NAME,
                params=mobility_params
            )
            
            # Send command
            command_id = self.command_client.robot_command(
                cmd,
                end_time_secs=time.time() + total_time
            )
            
            self.logger.info('Square trajectory sent, waiting for completion...')
            block_for_trajectory_cmd(self.command_client, command_id, timeout_sec=total_time + 5)
            self.logger.info('Square trajectory complete!')
            
            return True
            
        except Exception as e:
            self.logger.error(f'Square trajectory failed: {e}')
            return False
