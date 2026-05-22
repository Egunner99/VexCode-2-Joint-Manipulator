from vex import *
import math
import urandom


# Mechanical Constants
# ------------------------------------------------------------


ELBOW_RATIO = 24
SHOULDER_RATIO = 72


DIST_BUFFER = 10
L1 = 11     # shoulder length
L2 = 12     # elbow length


shoulder_motor.set_velocity(60, PERCENT)
elbow_motor.set_velocity(40, PERCENT)


# Angle Helpers
# ------------------------------------------------------------


def get_elbow_angle():
    return elbow_motor.position(DEGREES) / ELBOW_RATIO


def get_shoulder_angle():
    return shoulder_motor.position(DEGREES) / SHOULDER_RATIO


def set_elbow_angle(angle):
    target = angle * ELBOW_RATIO
    elbow_motor.spin_to_position(target, DEGREES, wait=False)


    while True:
        x, y, sx, sy = forward_kinematic()
        if y <= 0 or sy <= 0:
            elbow_motor.stop()
            break
        if elbow_motor.is_done():
            break


def set_shoulder_angle(angle):
    target = angle * SHOULDER_RATIO
    shoulder_motor.spin_to_position(target, DEGREES, wait=False)


    while True:
        x, y, sx, sy = forward_kinematic()
        if y <= 0 or sy <= 0 or limit_1.pressing():
            shoulder_motor.stop()
            break
        if shoulder_motor.is_done():
            break


# Calibration
# ------------------------------------------------------------


def calibrate_shoulder():
    while not limit_1.pressing():
        shoulder_motor.spin(REVERSE)


    shoulder_motor.stop()
    shoulder_motor.set_position(0, DEGREES)


    shoulder_motor.spin_to_position(3600, DEGREES)


def calibrate_elbow():
    while not limit_2.pressing():
        elbow_motor.spin(FORWARD)


    elbow_motor.stop()
    elbow_motor.set_position(2750, DEGREES)


def full_calibrate():
    while True:
        angle = potentiometer.angle(DEGREES)
        if angle < 220:
            shoulder_motor.spin(FORWARD)
        elif angle > 230:
            shoulder_motor.spin(REVERSE)
        else:
            shoulder_motor.stop()
            break


    calibrate_elbow()
    calibrate_shoulder()


# Kinematics
# ------------------------------------------------------------


def forward_kinematic(theta1=1e7, theta2=1e7):
    if theta1 >= 1e7:
        theta1 = get_shoulder_angle()
        theta2 = get_elbow_angle()


    alpha = math.pi - math.radians(theta1) - math.radians(theta2)


    shoulder_x = L1 * math.cos(math.radians(theta1))
    shoulder_y = L1 * math.sin(math.radians(theta1)) + 1.25


    x = shoulder_x - (L2 * math.cos(alpha))
    y = shoulder_y + (L2 * math.sin(alpha))


    return x, y, shoulder_x, shoulder_y


def euclidean(a, b):
    return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)


def get_distance():
    return distance_sensor.object_distance(INCHES) + DIST_BUFFER


def inverse_kinematic():
    x = get_distance()
    y = 1


    # theta2
    c2 = (x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2)
    theta2 = math.acos(c2)


    # theta1
    k1 = math.atan(y / x)
    k2 = math.asin((L2 * math.sin(theta2)) / math.sqrt(x*x + y*y))


    theta1 = math.degrees(k1 + k2)
    theta2 = math.degrees(theta2)


    # choose best elbow configuration
    fk1 = forward_kinematic(theta1, theta2)[:2]
    fk2 = forward_kinematic(theta1, -theta2)[:2]


    if euclidean(fk1, (x, y)) <= euclidean(fk2, (x, y)):
        return theta1, theta2
    else:
        return theta1, -theta2


# Dump Routine
# ------------------------------------------------------------


def dump():
    servo_claw.set_position(50)
    wait(1, SECONDS)


    set_shoulder_angle(70)
    set_elbow_angle(-45)


    set_shoulder_angle(50)
    set_elbow_angle(90)


    servo_claw.set_position(-50)
    wait(1, SECONDS)


# Main Loop
# ------------------------------------------------------------


full_calibrate()


while True:


    while get_distance() >= 23:
        wait(1, SECONDS)


    theta1, theta2 = inverse_kinematic()


    if get_distance() <= 18:
        set_shoulder_angle(theta1 + 5)


    set_elbow_angle(theta2)
    set_shoulder_angle(theta1)


    dump()


