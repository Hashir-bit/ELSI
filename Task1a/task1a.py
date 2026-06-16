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

# ── PID state (persists across control_loop calls) ────────────────────────────
_prev_error  = 0.0
_integral    = 0.0
_on_white_bg = False

# ── Tuning parameters ─────────────────────────────────────────────────────────
Kp           = 1.2    # proportional gain
Ki           = 0.0    # integral gain      (keep 0 until P+D are stable)
Kd           = 0.8    # derivative gain    (damps oscillation)

BASE_SPEED   = 2.5    # forward speed on straight sections
MAX_SPEED    = 5.0    # motor saturation cap
INTEGRAL_CAP = 2.0    # prevents integral windup

# Sensor position weights (left-most = -2, centre = 0, right-most = +2)
_WEIGHTS = {
    'left_corner':  -2.0,
    'left':         -1.0,
    'middle':        0.0,
    'right':        +1.0,
    'right_corner': +2.0,
}


def control_loop(sensors):
    global _prev_error, _integral, _on_white_bg

    avg = sum(sensors[k] for k in _WEIGHTS) / 5.0

    # Hysteresis: switch only when clearly past the threshold
    if avg > 0.6:
        _on_white_bg = True
    elif avg < 0.4:
        _on_white_bg = False

    adjusted = {k: 1.0 - sensors[k] for k in _WEIGHTS} if _on_white_bg else sensors

    # 1. Weighted-average error
    weighted_sum  = sum(adjusted[k] * w for k, w in _WEIGHTS.items())
    total_reading = sum(adjusted[k] for k in _WEIGHTS)

    if total_reading < 0.05:
        error = _prev_error
    else:
        error = weighted_sum / total_reading

    # 2. PID
    _integral  = max(-INTEGRAL_CAP, min(INTEGRAL_CAP, _integral + error))
    derivative = error - _prev_error
    _prev_error = error

    correction = Kp * error + Ki * _integral + Kd * derivative

    # 3. Differential drive
    left  = max(-MAX_SPEED, min(MAX_SPEED, BASE_SPEED + correction))
    right = max(-MAX_SPEED, min(MAX_SPEED, BASE_SPEED - correction))

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

            left, right = control_loop(last_sensors)
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