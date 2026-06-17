"""
===================================================
    eLSI Sprint 1 - Task 1A : PID Line Following
===================================================

Participant template.

HOW TO RUN
  1. Open the Task 1A scene in CoppeliaSim.
  2. Start the bridge:   python3 bridge_task1a.py --eval
  3. Run this file:      python3 task1a_template.py

WHAT YOU IMPLEMENT
  Only control_loop(). Everything else (connecting, receiving sensors,
  sending motor commands) is handled for you by CoppeliaClient.
  Don't Edit this file except control_loop().
  You can add helper functions if you like.

Team ID: [ 20 ]
"""

import time

from connector_task1a import CoppeliaClient

# The five line sensors, ordered left -> right across the robot.
# Each value is in [0.0, 1.0]; a higher value means the line is detected.
SENSOR_ORDER = ['left_corner', 'left', 'middle', 'right', 'right_corner']

# pid variables to remember last state
prev_error = 0.0
integral = 0.0

# to track which surface robot is on
on_white = False
white_locked = False
white_count = 0

lost_count = 0
done = False
finish_count = 0

# pid gains, tuned by trial and error
Kp = 1.5
Ki = 0.0
Kd = 0.9

base_speed = 2.5
max_speed = 5.0

# each sensor gets a position value, middle is 0, left is negative, right is positive
weights = {
    'left_corner':  -2.0,
    'left':         -1.0,
    'middle':        0.0,
    'right':        +1.0,
    'right_corner': +2.0,
}


def get_speed(error, adjusted):
    # slow down if corner sensors are active or error is too big (means sharp turn)
    corners = adjusted['left_corner'] + adjusted['right_corner']
    if corners > 0.8 or abs(error) > 1.2:
        return 1.0
    elif corners > 0.4 or abs(error) > 0.6:
        return 1.8
    else:
        return base_speed


def check_finish(adjusted):
    # if all 5 sensors see the line at same time its the finish bar
    return all(adjusted[k] > 0.6 for k in weights)


def control_loop(sensors):
    """Return (left_speed, right_speed) for the current sensor reading.

    `sensors` is a dict, e.g.:
        {'left_corner': 0.02, 'left': 0.41, 'middle': 0.95,
         'right': 0.05, 'right_corner': 0.01}

    ------------------------------------------------------------------
    TODO (participants): implement your PID line-following controller.
    ------------------------------------------------------------------
    A typical approach:
      1. Turn the 5 readings into ONE line-position error
      2. Feed that error through a PID controller:
      3. Drive the wheels differentially:
    """
    global prev_error, integral
    global on_white, white_locked, white_count
    global lost_count, done, finish_count

    if done:
        return 0.0, 0.0

    # average of all sensors to know if background is white or black
    avg = sum(sensors[k] for k in weights) / 5.0

    # if avg is high means white background, lock it so it dont switch back
    if not white_locked:
        if avg > 0.60:
            white_count += 1
        else:
            white_count = 0

        if white_count >= 2:
            on_white = True
            white_locked = True

    # on white background the black line reads low so we flip the values
    if on_white:
        adjusted = {k: 1.0 - sensors[k] for k in weights}
    else:
        adjusted = sensors

    # check for finish line only after we crossed to white side
    if white_locked and check_finish(adjusted):
        finish_count += 1
        if finish_count >= 3:
            done = True
            print("Finish line detected - stopping!")
            return 0.0, 0.0
    else:
        finish_count = 0

    weighted_sum = sum(adjusted[k] * w for k, w in weights.items())
    total = sum(adjusted[k] for k in weights)

    line_found = total >= 0.15

    if not line_found:
        lost_count += 1
    else:
        lost_count = 0

    # if line is lost for few ticks, rotate towards last known direction
    if lost_count > 3:
        t = 1.0
        if prev_error > 0:
            left, right = t, -t   # line was on right so turn right
        else:
            left, right = -t, t   # line was on left so turn left
        return left, right

    error = weighted_sum / total

    integral = max(-2.0, min(2.0, integral + error))
    derivative = error - prev_error
    prev_error = error

    correction = Kp * error + Ki * integral + Kd * derivative

    speed = get_speed(error, adjusted)

    left  = max(-max_speed, min(max_speed, speed + correction))
    right = max(-max_speed, min(max_speed, speed - correction))

    return left, right


def main():
    client = CoppeliaClient(host="127.0.0.1", port=50002)
    client.connect()
    print("Connected to bridge_task1a. Running... (Ctrl+C to stop)")

    last_sensors = None
    try:
        while True:
            # Pull the freshest sensor packet; reuse the last one between packets.
            sensors = client.receive_sensor_data()
            if sensors is not None:
                last_sensors = sensors
            if last_sensors is None:
                time.sleep(0.02)
                continue

            left, right = control_loop (last_sensors)
            client.send_motor_command(left, right)

            time.sleep(0.05)   # ~20 Hz control loop
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        try:
            client.send_motor_command(0.0, 0.0)   # stop the robot
        except Exception:
            pass
        client.close()


if __name__ == "__main__":
    main()
