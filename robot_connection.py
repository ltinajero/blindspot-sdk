"""Robot connection and authentication module.

This module handles all the initial setup needed to connect to and authenticate
with a Spot robot using the Boston Dynamics SDK.
"""

import logging
import os
import sys

import bosdyn.client
import bosdyn.client.util
from bosdyn.client import create_standard_sdk
from bosdyn.client.robot_command import RobotCommandClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.estop import EstopClient
from bosdyn.client.power import PowerClient


class SpotRobotConnection:
    """Manages connection, authentication, and basic control of a Spot robot."""

    def __init__(self, hostname, client_name='SpotClient', verbose=False):
        """Initialize robot connection parameters.
        
        Args:
            hostname: IP address or hostname of the Spot robot
            client_name: Name identifier for this client
            verbose: Enable verbose logging
        """
        self.hostname = hostname
        self.client_name = client_name
        self.verbose = verbose
        
        # Setup logging
        bosdyn.client.util.setup_logging(verbose)
        self.logger = logging.getLogger(client_name)
        
        # Initialize SDK and robot objects
        self.sdk = None
        self.robot = None
        self.lease_client = None
        self.lease_keepalive = None
        self.robot_command_client = None
        self.robot_state_client = None
        self.power_client = None
        
    def connect(self):
        """Create SDK and robot objects, establish connection.
        
        Raises:
            Exception: If connection fails
        """
        try:
            self.logger.info(f'Connecting to robot at {self.hostname}...')
            
            # Create SDK and robot
            self.sdk = create_standard_sdk(self.client_name)
            self.robot = self.sdk.create_robot(self.hostname)
            
            self.logger.info('Connection established.')
            return True
        except Exception as e:
            self.logger.error(f'Failed to connect: {e}')
            return False
    
    def authenticate(self, username='admin', password='password'):
        """Authenticate to the robot.
        
        Args:
            username: Robot username (default: 'admin')
            password: Robot password (default: 'password')
        
        Raises:
            Exception: If authentication fails
        """
        try:
            self.logger.info('Authenticating with robot...')
            
            # Try authentication with provided credentials
            try:
                self.robot.authenticate(username, password)
            except bosdyn.client.auth.InvalidLoginError:
                # Fall back to interactive prompt if credentials fail
                self.logger.warning('Credentials failed, using interactive login...')
                bosdyn.client.util.authenticate(self.robot)
            
            self.logger.info('Authentication successful.')
            return True
        except Exception as e:
            self.logger.error(f'Authentication failed: {e}')
            return False
    
    def setup_clients(self):
        """Initialize necessary service clients.
        
        Raises:
            Exception: If client setup fails
        """
        try:
            self.logger.info('Setting up service clients...')
            
            # Get essential clients
            self.lease_client = self.robot.ensure_client(
                LeaseClient.default_service_name)
            self.robot_command_client = self.robot.ensure_client(
                RobotCommandClient.default_service_name)
            self.robot_state_client = self.robot.ensure_client(
                RobotStateClient.default_service_name)
            self.power_client = self.robot.ensure_client(
                PowerClient.default_service_name)
            
            self.logger.info('Clients ready.')
            return True
        except Exception as e:
            self.logger.error(f'Failed to setup clients: {e}')
            return False
    
    def acquire_lease(self):
        """Acquire a lease from the robot (required to send commands).
        
        Raises:
            Exception: If lease acquisition fails
        """
        try:
            self.logger.info('Acquiring lease...')
            self.lease_keepalive = LeaseKeepAlive(
                self.lease_client, must_acquire=True, return_at_exit=True)
            self.logger.info('Lease acquired.')
            return True
        except Exception as e:
            self.logger.error(f'Failed to acquire lease: {e}')
            return False
    
    def release_lease(self):
        """Release the lease if held."""
        if self.lease_keepalive:
            self.logger.info('Releasing lease...')
            self.lease_keepalive.__exit__(None, None, None)
            self.lease_keepalive = None
    
    def verify_not_estopped(self):
        """Verify robot is not estopped.
        
        Returns:
            bool: True if not estopped, False otherwise
        """
        if self.robot.is_estopped():
            self.logger.error('Robot is estopped! Configure E-Stop before proceeding.')
            return False
        return True
    
    def power_on(self, timeout_sec=20):
        """Power on the robot.
        
        Args:
            timeout_sec: Timeout in seconds
        
        Raises:
            Exception: If power on fails
        """
        try:
            self.logger.info('Powering on robot...')
            self.robot.power_on(timeout_sec=timeout_sec)
            self.logger.info('Robot powered on.')
            return True
        except Exception as e:
            self.logger.error(f'Power on failed: {e}')
            return False
    
    def time_sync(self):
        """Wait for time synchronization with robot.
        
        Raises:
            Exception: If time sync fails
        """
        try:
            self.logger.info('Syncing time with robot...')
            self.robot.time_sync.wait_for_sync()
            self.logger.info('Time synchronized.')
            return True
        except Exception as e:
            self.logger.error(f'Time sync failed: {e}')
            return False
    
    def disconnect(self):
        """Disconnect from robot and cleanup."""
        self.logger.info('Disconnecting from robot...')
        self.release_lease()
        if self.robot:
            self.robot.shutdown()
        self.logger.info('Disconnected.')
