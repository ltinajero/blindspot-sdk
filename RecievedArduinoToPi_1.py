import os
import time
import serial
import sys
print(sys.version)   # confirms correct python version (3.10.13)
robot_ip = "192.168.80.3"
SERIAL_PORT = "/dev/ttyACM0"
BAUD = 9600
THRESHOLD = 1
DEBOUNCE_SEC = 0.4

if __name__ == "__main__":
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
    ser.reset_input_buffer()
    print("Press button once to start Square Walk...")

    last_pressed = False
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

        pressed_now = (value >= THRESHOLD)

        # rising edge = new press
        if pressed_now and not last_pressed:
            now = time.time()
            if now - last_press_time >= DEBOUNCE_SEC:
                last_press_time = now
                print("Starting square_walk_interrupt.py ...")

                # release serial port before launching the other program
                ser.close()

                # Replace THIS process with the square walk program (no second serial reader running)
                os.execvp(sys.executable, [
                    sys.executable,
                    "square_walk_interrupt.py",
                    robot_ip,
                ])

        last_pressed = pressed_now