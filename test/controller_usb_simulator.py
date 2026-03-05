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
from datetime import datetime


def main():
    parser = argparse.ArgumentParser(description="USB controller simulator")
    parser.add_argument("--port", required=True, help="serial port to open")
    parser.add_argument("--baud", type=int, default=115200, help="baud rate")
    parser.add_argument("--interval", type=float, default=0.01, help="orientation send interval (s)")
    parser.add_argument("-v", "--verbose", action="store_true", help="verbose debug output")
    parser.add_argument("--log", help="save all serial data to a log file")
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    print(f"Opened {args.port} at {args.baud} baud")
    if args.verbose:
        print("Verbose mode ON - showing all serial activity")
    if args.log:
        print(f"Logging to {args.log}")
    print("Sending orientation packets... (press Ctrl+C to exit)\n")
    
    logfile = None
    if args.log:
        logfile = open(args.log, 'w')
        logfile.write(f"HIL Serial Log - Started {datetime.now().isoformat()}\n")
        logfile.write(f"Port: {args.port}, Baud: {args.baud}\n")
        logfile.write("=" * 60 + "\n")

    try:
        iteration = 0
        while True:
            iteration += 1
            # generate a simple oscillating orientation or random for testing
            pitch = 10.0 * random.uniform(-1, 1)
            roll = 10.0 * random.uniform(-1, 1)
            yaw = random.uniform(0, 360)
            line = f"O: {pitch:.2f} {roll:.2f} {yaw:.2f}\n"
            ser.write(line.encode('ascii'))
            if args.verbose:
                msg = f"[{iteration}] Sent: {line.strip()}"
                print(msg)
                if logfile:
                    logfile.write(msg + "\n")
            
            # read any motor data
            received_data = False
            while True:
                data = ser.readline().decode('ascii', errors='ignore')
                if not data:
                    break
                received_data = True
                data = data.strip()
                if args.verbose or data:
                    msg = f"  Recv: {data}"
                    print(msg)
                    if logfile:
                        logfile.write(msg + "\n")
                
                if data.startswith('M:'):
                    parts = data[2:].strip().split()
                    try:
                        speeds = [float(p) for p in parts]
                        msg = f"  ✓ Motor speeds: {[f'{s:.1f}' for s in speeds]}"
                        print(msg)
                        if logfile:
                            logfile.write(msg + "\n")
                    except ValueError:
                        msg = f"  ✗ Failed to parse motor speeds: {data}"
                        print(msg)
                        if logfile:
                            logfile.write(msg + "\n")
                elif data and not data.startswith('M:'):
                    msg = f"  ? Unknown message: {data}"
                    print(msg)
                    if logfile:
                        logfile.write(msg + "\n")
            
            if logfile:
                logfile.flush()

            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        if logfile:
            logfile.close()
        ser.close()


if __name__ == '__main__':
    main()
