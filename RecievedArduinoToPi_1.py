import subprocess
import time
import sys

print(sys.version)   # confirms correct python version (3.10.13)
robot_ip = "192.168.80.3"

if __name__ == "__main__":
    print("Left button (1) starts square walk.")
    print("Right button (0) interrupts and makes Spot sit.")
    
    # Launch square_walk_interrupt.py as a subprocess
    # It will handle all button logic internally
    process = subprocess.Popen([
        sys.executable,
        "square_walk_interrupt.py",
        robot_ip,
        "--start_code",
        "1",
        "--stop_code",
        "0",
    ])
    
    # Wait for the subprocess to finish
    process.wait()