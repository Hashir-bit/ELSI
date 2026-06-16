"""
===================================================
    eLSI Sprint 1- [eLSI 2026-27]
===================================================

This script is intended to be a Boilerplate for
Bonus Task 0 of eLSI Sprint 1- [eLSI 2026-27]

Filename:        task0.py
Created:         29/05/2026
Last Modified:   29/05/2026
Author:          e-Yantra Team
Team ID:         [ 20 ]
This software is made available on an "AS IS WHERE IS BASIS".
Licensee/end user indemnifies and will keep e-Yantra indemnified from
any and all claim(s) that emanate from the use of the Software or
breach of the terms of this agreement.

e-Yantra - An MHRD project under National Mission on Education using ICT (NMEICT)
*****************************************************************************************
"""

import socket
import threading
import time


class SocketClient:
    """Holds socket client data and sensor information."""

    def __init__(self):
        self.sock = None
        self.running = False
        self.sensor_values = [0.0] * 32
        self.sensor_count = 0
        self.recv_thread = None
        self.control_thread = None


client = SocketClient()


def connect_to_server(c, ip, port):
    """
    Establishes connection to the CoppeliaSim server.

    :param c: SocketClient instance
    :param ip: IP address of the server (typically "127.0.0.1" for localhost)
    :param port: Port number of the server (typically 50002)
    :return: True if connection successful, False if failed
    """
    try:
        c.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    except OSError:
        print("Socket creation failed")
        return False

    try:
        c.sock.connect((ip, port))
    except OSError:
        print("Connection failed")
        c.sock.close()
        c.sock = None
        return False

    c.running = True

    c.recv_thread = threading.Thread(target=receive_loop, args=(c,), daemon=True)
    c.recv_thread.start()

    return True


def disconnect(c):
    """
    Cleanly disconnects from the server and cleans up resources.

    :param c: SocketClient instance
    """
    c.running = False

    if c.recv_thread is not None:
        c.recv_thread.join()

    if c.sock is not None:
        c.sock.close()
        c.sock = None


def set_motor(c, left, right):
    """
    Sends motor control commands to the robot.

    :param c: SocketClient instance
    :param left: Left motor speed ( where negative values reverse direction)
    :param right: Right motor speed ( where negative values reverse direction)

    Command format: "L:<left_speed>;R:<right_speed>\\n"
    Example: "L:0.5;R:0.3\\n" sets left motor to 50% forward, right motor to 30% forward
    """
    if c.sock is not None:
        cmd = "L:{:f};R:{:f}\n".format(left, right)
        try:
            c.sock.sendall(cmd.encode())
        except OSError:
            pass


def receive_loop(c):
    """
    Thread function that continuously receives sensor data from the server.

    This function runs in a separate thread and parses incoming sensor data.
    Expected data format: "S:<sensor1>,<sensor2>,<sensor3>,...\\n"
    Example: "S:0.125,0.0,1.0,0.5\\n" represents 4 sensor values
    """
    while c.running:
        try:
            data = c.sock.recv(2048)
        except OSError:
            data = b""

        if data:
            buffer = data.decode(errors="ignore")

            if buffer.startswith("S:"):
                values = buffer[2:]
                tokens = values.split(",")

                idx = 0
                for token in tokens:
                    if idx >= 32:
                        break
                    token = token.strip()
                    if token == "":
                        continue
                    try:
                        c.sensor_values[idx] = float(token)
                        idx += 1
                    except ValueError:
                        c.sensor_values[idx] = 0.0
                        idx += 1
                c.sensor_count = idx

        time.sleep(0.05)


def control_loop(c):
    """
    Main control loop thread for robot behavior.

    Time-based square path:
    - Move forward for one side
    - Turn left 90 degrees
    - Repeat 4 times
    - Stop at the starting point
    """

    FORWARD_SPEED = 2
    TURN_SPEED = 1.5

    SIDE_TIME = 18.01
    TURN_TIME = 2.28

    PAUSE_TIME = 0.20
    START_DELAY = 1.00

    def stop_robot():
        set_motor(c, 0.0, 0.0)

    print("Starting timed square path...")
    time.sleep(START_DELAY)

    for side in range(4):
        if not c.running:
            break

        print("Side {}: moving forward".format(side + 1))
        set_motor(c, FORWARD_SPEED, FORWARD_SPEED)
        time.sleep(SIDE_TIME)

        stop_robot()
        time.sleep(PAUSE_TIME)

        print("Corner {}: turning left 90 degrees".format(side + 1))
        set_motor(c, -TURN_SPEED, TURN_SPEED)
        time.sleep(TURN_TIME)

        stop_robot()
        time.sleep(PAUSE_TIME)

    stop_robot()
    print("Square complete! Robot stopped.")

    while c.running:
        time.sleep(0.1)


def main():
    """
    Main function - Entry point of the program.

    This function:
    1. Connects to the CoppeliaSim server
    2. Starts the control thread for robot behavior
    3. Continuously displays sensor data
    4. Handles cleanup when program exits
    """
    if not connect_to_server(client, "127.0.0.1", 50002):
        print("Failed to connect to CoppeliaSim server. Make sure:")
        print("1. CoppeliaSim is running")
        print("2. The simulation scene is loaded")
        print("3. The ZMQ remote API is enabled on port 50002")
        return -1

    print("Successfully connected to CoppeliaSim server!")
    print("Starting control thread...")

    client.control_thread = threading.Thread(target=control_loop, args=(client,), daemon=True)
    client.control_thread.start()

    print("Monitoring sensor data... (Press Ctrl+C to exit)")
    try:
        while True:
            if client.sensor_count > 0:
                values = " ".join("{:.3f}".format(client.sensor_values[i])
                                  for i in range(client.sensor_count))
                print("Sensors ({}): {} ".format(client.sensor_count, values))
            else:
                print("Waiting for sensor data...")

            time.sleep(0.2)
    except KeyboardInterrupt:
        print("Disconnecting...")
        disconnect(client)

    return 0


if __name__ == "__main__":
    main()