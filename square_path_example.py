#!/usr/bin/env python3
"""Example script to command Spot to walk in a square path.

This is the main entry point that demonstrates how to use the robot connection
and square path modules to command Spot to walk in a square pattern.

Usage:
    python3 square_path_example.py <ROBOT_IP> [--side-length 1.0] [--velocity 0.5]

Example:
    python3 square_path_example.py 192.168.1.20
    python3 square_path_example.py 192.168.1.20 --side-length 2.0 --velocity 0.3
"""

import argparse
import logging
import sys

from robot_connection import SpotRobotConnection
from square_movement import SquarePathWalker


def main(hostname, side_length=1.0, max_velocity=0.5, verbose=False):
    """Main function to run the square path example.
    
    Args:
        hostname: Robot IP address or hostname
        side_length: Length of square side in meters
        max_velocity: Maximum walking velocity in m/s
        verbose: Enable verbose logging
    
    Returns:
        int: 0 if successful, 1 if failed
    """
    
    logger = logging.getLogger('SquarePathExample')
    
    # Step 1: Connect to robot
    print(f'\n{"="*60}')
    print('SPOT SQUARE PATH EXAMPLE')
    print(f'{"="*60}\n')
    
    robot_conn = SpotRobotConnection(hostname, verbose=verbose)
    
    if not robot_conn.connect():
        return 1
    
    # Step 2: Authenticate
    if not robot_conn.authenticate():
        return 1
    
    # Step 3: Setup clients
    if not robot_conn.setup_clients():
        return 1
    
    # Step 4: Acquire lease
    if not robot_conn.acquire_lease():
        return 1
    
    try:
        # Step 5: Verify safety conditions
        if not robot_conn.verify_not_estopped():
            logger.error('Please configure E-Stop before running this example.')
            return 1
        
        # Step 6: Power on and sync time
        if not robot_conn.power_on():
            return 1
        
        if not robot_conn.time_sync():
            return 1
        
        # Step 7: Stand up
        walker = SquarePathWalker(
            robot_conn.robot_command_client,
            robot_conn.robot_state_client,
            logger=logger
        )
        
        if not walker.stand_up():
            return 1
        
        # Step 8: Walk square path
        print(f'\n{"="*60}')
        print(f'Starting square path walk:')
        print(f'  Side length: {side_length}m')
        print(f'  Max velocity: {max_velocity}m/s')
        print(f'{"="*60}\n')
        
        if not walker.walk_square(
            side_length=side_length,
            max_velocity=max_velocity,
            total_time=20.0
        ):
            return 1
        
        print(f'\n{"="*60}')
        print('SQUARE PATH COMPLETE!')
        print(f'{"="*60}\n')
        
        return 0
        
    except KeyboardInterrupt:
        logger.info('Interrupted by user.')
        return 1
    except Exception as e:
        logger.error(f'Unexpected error: {e}')
        return 1
    finally:
        # Always disconnect and cleanup
        robot_conn.disconnect()


def parse_arguments():
    """Parse command line arguments.
    
    Returns:
        Parsed arguments
    """
    parser = argparse.ArgumentParser(
        description='Command Spot to walk in a square path.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  python3 square_path_example.py 192.168.1.20
  python3 square_path_example.py spot.local --side-length 2.0
  python3 square_path_example.py 10.0.0.3 --side-length 1.5 --velocity 0.4 --verbose
        '''
    )
    
    parser.add_argument(
        'hostname',
        help='Hostname or IP address of the Spot robot'
    )
    
    parser.add_argument(
        '--side-length',
        type=float,
        default=1.0,
        help='Length of each side of the square in meters (default: 1.0)'
    )
    
    parser.add_argument(
        '--velocity',
        type=float,
        default=0.5,
        help='Maximum walking velocity in m/s (default: 0.5)'
    )
    
    parser.add_argument(
        '--verbose',
        action='store_true',
        help='Enable verbose logging'
    )
    
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_arguments()
    
    # Validate inputs
    if args.side_length <= 0:
        print('Error: side-length must be positive')
        sys.exit(1)
    
    if args.velocity <= 0 or args.velocity > 2.0:
        print('Error: velocity must be between 0 and 2.0 m/s')
        sys.exit(1)
    
    sys.exit(main(
        hostname=args.hostname,
        side_length=args.side_length,
        max_velocity=args.velocity,
        verbose=args.verbose
    ))
