import subprocess
import time
import sys

print(sys.version)   # confirms correct python version (3.10.13)
robot_ip = "192.168.80.3"

if __name__ == "__main__":
    print("Press button once to start Square Walk...")
    print("Press again while running to interrupt and sit...")
    
    # Launch square_walk_interrupt.py as a subprocess
    # It will handle all button logic internally
    process = subprocess.Popen([
        sys.executable,
        "square_walk_interrupt.py",
        robot_ip,
    ])
    
    # Wait for the subprocess to finish
    process.wait()