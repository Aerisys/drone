"""Simple Python script that emulates the Unity side of the ControllerUSB protocol.

Run this on the host PC with a serial connection to the ESP32 (e.g. /dev/ttyUSB0).
It periodically sends orientation data and prints any motor commands received
from the device.

Requires pyserial:
    pip install pyserial

Usage:
    python3 controller_usb_simulator.py --port /dev/ttyUSB0

"""

import argparse
import random
import time
import serial


def main():
    parser = argparse.ArgumentParser(description="USB controller simulator")
    parser.add_argument("--port", required=True, help="serial port to open")
    parser.add_argument("--baud", type=int, default=115200, help="baud rate")
    parser.add_argument("--interval", type=float, default=0.01, help="orientation send interval (s)")
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    print(f"Opened {args.port} at {args.baud} baud")

    try:
        while True:
            # generate a simple oscillating orientation or random for testing
            pitch = 10.0 * random.uniform(-1, 1)
            roll = 10.0 * random.uniform(-1, 1)
            yaw = random.uniform(0, 360)
            line = f"O: {pitch:.2f} {roll:.2f} {yaw:.2f}\n"
            ser.write(line.encode('ascii'))
            # read any motor data
            while True:
                data = ser.readline().decode('ascii', errors='ignore')
                if not data:
                    break
                data = data.strip()
                if data.startswith('M:'):
                    parts = data[2:].strip().split()
                    try:
                        speeds = [float(p) for p in parts]
                    except ValueError:
                        continue
                    print("Motor speeds from device:", speeds)
                else:
                    print("<unknown> ", data)

            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ser.close()


if __name__ == '__main__':
    main()
