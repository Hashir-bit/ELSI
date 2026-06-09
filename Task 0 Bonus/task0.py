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

    Strategy: Line-follow along the black square border.
    Sensors (indices 0-4):
      0 = left IR      1 = right IR      2 = middle IR
      3 = left-corner  4 = right-corner

    A sensor value < BLACK_THRESH means it is OVER the black line.

    Square logic:
      - Follow the line (middle sensor keeps us on track)
      - When a corner sensor fires → we've reached a corner
      - Stop, turn 90° until the middle sensor re-acquires the line
      - Repeat 4 times, then stop at the original start corner
    """

    # Enable TCP_NODELAY to disable network buffering and prevent concatenated packets
    try:
        import socket
        c.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except Exception:
        pass

    # ── Thresholds & speeds ──────────────────────────────────────
    BLACK_THRESH   = 0.12   # sensor value below this = on black line
    FORWARD_SPEED  = 1.2    # straight-line speed (slightly reduced for stability)
    TURN_SPEED     = 0.5    # in-place turn speed (slower for high precision)
    SLOW_SPEED     = 0.4    # correction speed while line-following
    LOOP_DT        = 0.05   # control loop period (seconds)
    # ─────────────────────────────────────────────────────────────

    # ── Local shadowed set_motor to prevent bridge TCP collisions ──
    _global_set_motor = globals()['set_motor']
    last_left = [None]
    last_right = [None]
    last_send_time = [0.0]

    def set_motor(client_obj, left, right):
        # Round to avoid float precision noise
        left = round(left, 3)
        right = round(right, 3)
        
        # Send ONLY if speeds changed
        if left != last_left[0] or right != last_right[0]:
            now = time.time()
            elapsed = now - last_send_time[0]
            # Enforce at least 50ms gap between consecutive socket writes
            if elapsed < 0.05:
                time.sleep(0.05 - elapsed)
            _global_set_motor(client_obj, left, right)
            last_left[0] = left
            last_right[0] = right
            last_send_time[0] = time.time()

    # Sensor validation state to filter out TCP/bridge parsing drops to 0.0
    last_valid_sensors = [0.6] * 5

    def update_sensors():
        current_sensors = [c.sensor_values[i] for i in range(5)]
        # Ignore completely zeroed out packets (connection drop or start buffer)
        if all(v == 0.0 for v in current_sensors):
            return False
        # Filter out spikes where an individual sensor drops to exactly 0.0 from a white tile value (> 0.45)
        for i in range(5):
            val = current_sensors[i]
            if val == 0.0 and last_valid_sensors[i] > 0.45:
                pass
            else:
                last_valid_sensors[i] = val
        return True

    def on_black(sensor_idx):
        return last_valid_sensors[sensor_idx] < BLACK_THRESH

    # ── Wait for first sensor packet ─────────────────────────────
    while c.sensor_count == 0:
        time.sleep(0.1)
    time.sleep(0.5)

    # Initialize sensor validation array with starting values
    for i in range(5):
        last_valid_sensors[i] = c.sensor_values[i]

    corners_done  = 0
    cooldown_until = 0.0
    last_seen_side = "left"
    print("Square path: line-following + corner detection…")

    # ── Leave start corner before counting begins ─────────────────
    set_motor(c, FORWARD_SPEED, FORWARD_SPEED)
    
    # Wait until both corner sensors and front three sensors clear the starting junction
    timeout = time.time() + 1.5
    while c.running and time.time() < timeout:
        update_sensors()
        if not on_black(3) and not on_black(4) and not (on_black(0) and on_black(1) and on_black(2)):
            break
        time.sleep(LOOP_DT)
    time.sleep(0.2)  # Extra padding to be fully clear

    while c.running and corners_done < 4:

        now = time.time()

        # Update and validate incoming sensor data
        if not update_sensors():
            time.sleep(LOOP_DT)
            continue

        mid   = on_black(2)
        left  = on_black(0)
        right = on_black(1)
        lc    = on_black(4)
        rc    = on_black(3)

        # ── Corner detected (with cooldown debounce) ──────────────
        # Triggers if a corner sensor fires OR if all three front sensors see the perpendicular line
        if (lc or rc or (left and right and mid)) and now > cooldown_until:
            corners_done += 1
            print("Corner {} reached! lc={} rc={} 3front={}".format(corners_done, lc, rc, left and right and mid))

            # 1. Stop at the corner
            set_motor(c, 0.0, 0.0)
            time.sleep(0.8)

            # 2. Nudge forward so wheels are directly on the corner junction
            print("Nudging forward onto corner…")
            set_motor(c, FORWARD_SPEED, FORWARD_SPEED)
            time.sleep(0.6)

            # 3. Stop before turning
            set_motor(c, 0.0, 0.0)
            time.sleep(0.8)

            # 4. Turn LEFT (counter-clockwise)
            print("Turning left 90°…")
            set_motor(c, -TURN_SPEED, TURN_SPEED)

            # Phase 1: blind turn to clear initial corner alignment
            time.sleep(0.5)

            # Phase 2: keep turning until the middle sensor is on the line
            timeout = time.time() + 3.0
            while c.running and time.time() < timeout:
                update_sensors()
                if on_black(2):
                    break
                time.sleep(LOOP_DT)

            # 5. Stop after turn
            set_motor(c, 0.0, 0.0)
            time.sleep(0.8)
            last_seen_side = "right"  # default recovery direction after left turn (in case of overshoot)

            if corners_done == 4:
                break

            # Cooldown: ignore corner triggers for 1.2 s after turning
            cooldown_until = time.time() + 1.2

        # ── Normal line-following ─────────────────────────────────
        elif mid and not left and not right:
            # Centered on line — go straight
            set_motor(c, FORWARD_SPEED, FORWARD_SPEED)

        elif left and not right:
            # Drifted right — steer left (slow left wheel to turn left)
            set_motor(c, SLOW_SPEED, FORWARD_SPEED)
            last_seen_side = "left"

        elif right and not left:
            # Drifted left — steer right (slow right wheel to turn right)
            set_motor(c, FORWARD_SPEED, SLOW_SPEED)
            last_seen_side = "right"

        elif left and right:
            # Both side sensors on line (wide line section) — go straight
            set_motor(c, FORWARD_SPEED, FORWARD_SPEED)

        else:
            # Lost the line — steer in the direction of the last seen side
            if last_seen_side == "left":
                set_motor(c, SLOW_SPEED, FORWARD_SPEED)
            else:
                set_motor(c, FORWARD_SPEED, SLOW_SPEED)

        time.sleep(LOOP_DT)

    # ── Done ──────────────────────────────────────────────────────
    set_motor(c, 0.0, 0.0)
    print("Square complete! Robot stopped at start corner.")
    c.running = False


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