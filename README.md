# VEX Robotic Arm
Developed by Giovanni Mayorga, Andrew Berman, Nino Kramer

Autonomous 2-DOF robotic arm built in VEXcode. Uses a distance sensor to detect objects, solves inverse kinematics to position the arm, and dumps the object into a target.

---

## Hardware

- Shoulder motor (gear ratio 72:1)
- Elbow motor (gear ratio 24:1)
- Servo claw
- Distance sensor (object detection)
- Potentiometer (shoulder calibration reference)
- Limit switch 1 — shoulder home
- Limit switch 2 — elbow home

---

## Behavior

**Calibration:** on startup, the arm runs `full_calibrate()`. This aligns the shoulder using the potentiometer, then homes both joints via limit switches.

**Waiting:** idles until an object is within 23 inches.

**Pick:** solves IK for the object's distance, positions the shoulder and elbow, closes the claw.

**Dump:** raises the arm to a fixed dump position, opens the claw, returns to ready.

---

## Tuning

```python
L1 = 11        # shoulder segment length (inches)
L2 = 12        # elbow segment length (inches)
DIST_BUFFER = 10  # added to raw sensor distance

shoulder_motor.set_velocity(60, PERCENT)
elbow_motor.set_velocity(40, PERCENT)
```

Object detection range:
```python
while get_distance() >= 23:   # wait until object is this close
```

Close-range angle compensation (activates under 18 in):
```python
if get_distance() <= 18:
    set_shoulder_angle(theta1 + 5)
```

---

## Setup

1. Configure all devices in VEXcode to match the names in the code
2. Mount the arm so the shoulder limit switch triggers at the true home position
3. Verify `DIST_BUFFER` accounts for any mechanical offset between the sensor and the claw
4. Download and run. The calibration happens automatically on start
