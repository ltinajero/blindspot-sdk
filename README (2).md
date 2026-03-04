# Spot Square Path Example

A minimal, clean example to connect to Boston Dynamics' Spot robot and command it to walk in a square pattern.

## Overview

This project demonstrates the essential steps needed to:
1. **Connect** to a Spot robot
2. **Authenticate** with credentials
3. **Acquire** a lease (required for sending commands)
4. **Power on** and prepare the robot
5. **Command** the robot to walk in a square path

## Directory Structure

```
.
├── robot_connection.py          # Core robot connection & authentication
├── square_movement.py           # Square path walking logic
├── square_path_example.py       # Main example script
├── requirements.txt             # Python dependencies
└── README.md                    # This file
```

## Prerequisites

### Hardware
- A Boston Dynamics Spot robot
- Network connectivity to the robot (check IP/hostname)
- E-Stop configured (required for safe operation)
- Robot motor enable button activated

### Software
- Python 3.7+
- pip (Python package manager)

## Installation

### 1. Clone or Copy This Template

```bash
# Copy this entire directory to your workspace
cp -r spot_starter_template ~/my-spot-project
cd ~/my-spot-project
```

### 2. Install Dependencies

#### Option A: Using prebuilt wheels from spot-sdk
If you have the spot-sdk downloaded locally:

```bash
# Create virtual environment (recommended)
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install bosdyn packages from prebuilt wheels
pip install /path/to/spot-sdk/prebuilt/bosdyn_client-5.1.0-py3-none-any.whl
pip install /path/to/spot-sdk/prebuilt/bosdyn_core-5.1.0-py3-none-any.whl
pip install /path/to/spot-sdk/prebuilt/bosdyn_api-5.1.0-py3-none-any.whl

# Install other requirements
pip install numpy
```

#### Option B: Install from PyPI
```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## Usage

### Basic Example

```bash
python3 square_path_example.py <ROBOT_IP_ADDRESS>
```

Example:
```bash
python3 square_path_example.py 192.168.1.20
```

### With Options

```bash
python3 square_path_example.py 192.168.1.20 \
    --side-length 2.0 \
    --velocity 0.4 \
    --verbose
```

### Command Line Options

- `hostname` (required): IP address or hostname of the Spot robot
- `--side-length`: Length of each side of the square in meters (default: 1.0m)
- `--velocity`: Maximum walking velocity in m/s (default: 0.5 m/s)
- `--verbose`: Enable detailed logging output

## How It Works

### 1. `robot_connection.py`

Handles all robot connection and setup:
- **`SpotRobotConnection`** class provides:
  - `connect()` - Establish connection to robot
  - `authenticate()` - Authenticate with username/password
  - `setup_clients()` - Initialize service clients
  - `acquire_lease()` - Get command authority (required!)
  - `power_on()` - Power up the robot
  - `time_sync()` - Sync internal clock with robot
  - `disconnect()` - Clean shutdown

### 2. `square_movement.py`

Commands the robot to walk:
- **`SquarePathWalker`** class provides:
  - `stand_up()` - Stand the robot upright
  - `walk_square()` - Walk a square pattern using waypoint commands
  - `walk_square_trajectory()` - Alternative: send all waypoints in one command

### 3. `square_path_example.py`

Main script that ties everything together:
1. Connects and authenticates
2. Sets up necessary clients
3. Acquires lease
4. Powers on and syncs time
5. Commands robot to stand
6. Commands square path walk
7. Safely disconnects

## Important Safety Notes

⚠️ **Before Running This Example:**

1. **E-Stop Required**: Ensure an external E-Stop is configured and connected via separate process or tablet
2. **Clear Space**: Robot needs ~3-4 meters of clear space in all directions
3. **Operator Present**: Always have an operator ready to intervene
4. **Flat Surface**: Robot should be on a level floor for trajectory commands
5. **Motor Enable**: Press the motor enable button on the rear panel
6. **Battery Charged**: Ensure robot has sufficient battery charge

## Common Issues & Troubleshooting

### Connection Issues
```python
# Error: "Failed to connect"
# Check: Robot IP address is correct, robot is powered on, network connectivity
```

### Authentication Failures
```python
# Error: "InvalidLoginError"
# Solution: Default username is 'admin', password is 'password'
# Or modify the code to use your robot's credentials
```

### "Robot is estopped" error
```python
# Solution: Use a separate E-Stop client or tablet to configure E-Stop
# See: spot-sdk/python/examples/estop/
```

### Lease acquisition failures
```python
# Error: "Failed to acquire lease"
# Reason: Another client may hold the lease
# Solution: Restart robot or wait for other client to release lease
```

### "Robot is not powered on"
```python
# Solution: Call power_on() before sending movement commands
# Already handled in example script
```

## Extending This Code

### Add Your Own Movements

In `square_movement.py`, add methods like:

```python
def walk_rectangle(self, width, height, velocity):
    """Walk a rectangular path"""
    # Similar to walk_square() but with different x/y values
    
def walk_circle(self, radius, velocity):
    """Walk a circular path"""
    # Use sin/cos to generate circular waypoints
```

### Customize Robot Behavior

Modify `square_path_example.py`:
- Change `side_length` and `velocity` parameters
- Add arm movements (see `arm_and_mobility_command` in full SDK)
- Add gripper control
- Add perception-based navigation

## Key Imports from Boston Dynamics SDK

```python
# Basic connection
from bosdyn.client import create_standard_sdk
from bosdyn.client.robot_command import RobotCommandClient

# Utilities
from bosdyn.client.util import authenticate, setup_logging
from bosdyn.client.frame_helpers import VISION_FRAME_NAME, get_vision_tform_body

# Commands
from bosdyn.client.robot_command import RobotCommandBuilder, blocking_stand
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.power import PowerClient
```

## API Documentation

For complete SDK documentation, refer to:
- `spot-sdk/docs/` directory
- [Boston Dynamics Spot SDK Documentation](https://dev.bostondynamics.com/docs/concepts/index)
- Python docstrings in bosdyn-client package

## Next Steps

1. ✅ Run this example successfully
2. 📖 Read the full Quickstart Guide: `spot-sdk/docs/python/quickstart.md`
3. 🔧 Explore other examples in `spot-sdk/python/examples/`
4. 📚 Study the API documentation
5. 🤖 Build your custom application!

## License

This example code follows the Boston Dynamics Software Development Kit License agreement. See the spot-sdk LICENSE file for details.

## Support

For issues with the Boston Dynamics SDK:
- Check the spot-sdk examples directory
- Review SDK documentation in spot-sdk/docs
- Contact Boston Dynamics support

---

**Happy robot programming! 🤖**
