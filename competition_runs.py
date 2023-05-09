from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.pupdevices import Remote
from pybricks.parameters import Button
from pybricks.parameters import Port, Direction
from pybricks.robotics import GyroDriveBase

# import time
# import math
# class IODeviceKind(58): ...

hub = PrimeHub()

# __________________________________________________________________________________________________
# Initialize
# __________________________________________________________________________________________________

# Robot Constants
W_DIAMETER = 61.6  # mm
W_DISTANCE = 91  # mm
W_CIRC = W_DIAMETER * 3.1415926535  # mm
WHEELS_WALL_RATIO = 4  # TODO
WALL_MOTOR_RATIO = 35 / 6  # TODO

# Hardware Definition
hub = PrimeHub()
leftW = Motor(Port.F, positive_direction=Direction.COUNTERCLOCKWISE)
rightW = Motor(Port.B)
wall = Motor(Port.E, gears=[24, 140])
frontCS = ColorSensor(Port.D)
backCS = ColorSensor(Port.A)
wheels = GyroDriveBase(leftW, rightW, wheel_diameter=W_DIAMETER, axle_track=W_DISTANCE)

COLOR_LIST = [
    Color.WHITE,
    Color.BLACK,
    Color.GREEN,
    Color.RED,
    Color.BLUE,
    Color.YELLOW,
    Color.MAGENTA,
    Color.NONE,
]
frontCS.detectable_colors(COLOR_LIST)
backCS.detectable_colors(COLOR_LIST)

TARGET = 60
BLACK = 20
WHITE = 98

# Reset Wheels
leftW.reset_angle(0)
rightW.reset_angle(0)

# Print info
print(f"Battery: V {hub.battery.voltage()} C {hub.battery.current()}")
print(
    f"Drivebase (Dist): S {wheels.distance_control.limits()[0]} A {wheels.distance_control.limits()[1]} T {wheels.distance_control.limits()[2]}"
)
print(
    f"Drivebase (Head): S {wheels.heading_control.limits()[0]} A {wheels.heading_control.limits()[1]} T {wheels.heading_control.limits()[2]}"
)
print(
    f"Left W: S {leftW.control.limits()[0]} A {leftW.control.limits()[1]} T {leftW.control.limits()[2]}"
)
print(
    f"Right W: S {rightW.control.limits()[0]} A {rightW.control.limits()[1]} T {rightW.control.limits()[2]}"
)
print(
    f"Wall: S {wall.control.limits()[0]} A {wall.control.limits()[1]} T {wall.control.limits()[2]}"
)


class PetroError(Exception):
    ...


# _______________________________________________________________________________________________________________
# Run Functions
# _______________________________________________________________________________________________________________


def green_run():
    # initialize
    reset()
    arm = Motor(Port.C)
    arm.reset_angle(0)

    # TV Mission
    # gyro_follow(300, 11, angle=0, stop=False)
    wheels.settings(straight_speed=400)
    wheels.straight(40, then=Stop.NONE)
    accelerate(2.1, 400, 100, stop=True)  # slows before tv
    wheels.straight(-135)
    no_wall_turn(-45, speed=90)  # no wall turn to the rechargable battery
    wheels.settings(straight_speed=350)
    wheels.straight(510)

    # WIND TURBINE
    turn_to_angle(45)  # turns to the wind turbine
    wall_turn(0)  # reset the wall
    wheels.straight(360)

    for i in range(3):
        wait(500)
        wheels.straight(-80)  # backwards
        if hub.imu.heading() > 50 or hub.imu.heading() < 40:
            turn_to_angle(45)
        wait(800)
        rightW.run(700)  # TODO
        leftW.run(660)
        wait(750)
        leftW.hold()
        rightW.hold()

    # HYBRID CAR
    turn_to_angle(45)
    wheels.straight(-245, then=Stop.HOLD, wait=True)  # back from wind turbine
    # turn_to_angle(50)
    # wait(1000)
    # wheels.straight(-70)
    turn_to_angle(115)  # turn towards hybrid car
    wall_turn(0)
    wheels.straight(-210)
    arm.run_angle(900, 120, then=Stop.HOLD, wait=True)  # reset arm
    turn_to_angle(142)
    wheels.straight(-130)
    arm.run_angle(1000, -150, then=Stop.HOLD, wait=True)  # do mission
    arm.run_angle(900, 130, then=Stop.HOLD, wait=True)

    # # HOMEWARDS
    wheels.straight(450)
    turn_to_angle(150)
    wheels.drive(800, 0)
    while True:
        pass


def red_run():
    reset()
    wheels.drive(330, -5)  # back from the factory
    wait(2000)
    # gyro_time(300, 2, 0)  # drive to the factory
    wheels.straight(-190)  # back from the factory

    # Power Plant
    no_wall_turn(-45)
    wall_turn(90)  # positioning to drive to black line
    wheels.drive(350, 0)
    wait(700)
    gyro_until_black(
        350, angle=-38, kp=1.2, sensor=frontCS, stop=False
    )  # drive to black line

    wheels.straight(
        60,
    )  # center robot on black line
    no_wall_turn(45)  # turn in place
    wheels.drive(250, 0)  # drive until green
    # sees_green = False
    while not frontCS.color() == Color.GREEN:
        ...
    # rgb = frontCS.get_rgb_intensity()
    # sees_green = (rgb[1] - ((rgb[0] + rgb[2]) / 2) > 75) and rgb[
    #     3
    # ] < 800  # Green is 75 higher than average of red and blue, excluding white

    leftW.hold()
    rightW.hold()
    wall_turn(90)  # turn wall to position
    follow_line(
        -150, 105, kp=3, side="right", sensor=backCS, stop=False
    )  # align trigger
    # towards power plant
    wheels.stop()

    rightW.run_time(-700, 1500, then=Stop.HOLD, wait=False)
    leftW.run_time(-700, 1500, then=Stop.HOLD, wait=True)

    # if hub.imu.heading() >= 50:
    #     rightW.run_angle(200, 35)

    # reverse from power plant
    leftW.run_time(250, 800, then=Stop.HOLD, wait=True)
    rightW.run_time(250, 800, then=Stop.HOLD, wait=True)
    # Smart Electricity Grid
    wheels.straight(100)
    # turn_to_angle(50)  # turn to grid

    wall_turn(90)  # position grabbing attachment
    wheels.straight(550)
    gyro_follow(350, 20, 50, kp=0.5)
    # accelerate(0.5, 400, 200)  # grab
    wheels.stop()

    # # Water Units
    # gyro_until_black(-35, hub.imu.heading())  # back until line
    # gyro_follow(-45, 1, 46, kp=0.5)  # back
    # turn_to_angle(-45)  # turn towards water units
    # gyro_follow(45, 6.5, -55, kp=0.5)  # drive towards them
    # gyro_follow(-75, 3, -55, kp=0.5)  # back
    # turn_to_angle(0)  # turn to line

    # # Oil Rig
    # gyro_until_black(30, 1)  # back until line
    # gyro_follow(40, 2.5, 30, kp=0.5)  # center on line
    # turn_until_black(25)
    # follow_line(40, 35, kp=0.4, side="left", sensor=frontCS)  # Drive to oil rig
    # wall_turn(50)  # Position ramp
    # gyro_follow(50, 9, -43, kp=0.5)  # drive towards them

    # for i in range(3):  # Push oil rig
    #     # drive back
    #     leftW.run_time(-60, 400, then=Stop.HOLD, wait=True)
    #     rightW.run_time(-60, 400, then=Stop.HOLD, wait=True)
    #     # drive forward
    #     leftW.run_time(70, 600, then=Stop.HOLD, wait=True)
    #     rightW.run_time(70, 600, then=Stop.HOLD, wait=True)
    # # Back to base
    # wheels.straight(80)
    # no_wall_turn(-105)
    # wheels.drive(80, 0)

    # wait(3000)
    # while True:
    #     pass


def violet_run():
    reset()
    gyro_follow(60, 6.5, angle=0, kp=2, stop=True)  # forward to m11
    wall_turn(270)  # turn wall to move enrgay unite
    wheels.drive(40, 0)  # take unite and water unite
    wait(1000)  # wait fir water to drop
    wheels.drive(-100, 0)
    while True:
        pass


def blue_run():
    reset()
    # relissing 3 untis to magar enrgia
    gyro_until_black(45, angle=0, kp=2, sensor=frontCS, stop=True)  # לתחילת הקו השחור
    wheels.straight(90)
    wall.run_angle(180, 45, then=Stop.HOLD, wait=True)
    turn_to_angle(38)
    follow_line_time(
        35, 3.5, kp=0.5, side="left", sensor=frontCS, stop=True
    )  # towarsds energy storage
    # gyro_time(40,0.7 ,0)

    # solar farm
    gyro_follow(-35, 2, angle=3, kp=2, stop=True)  # back from mager enrgia
    turn_to_angle(90)  # turn
    gyro_follow(40, 5.5, angle=91, kp=2, stop=True)  # twords enrgy untis in solar farms
    wall.run_angle(180, -135, then=Stop.HOLD, wait=True)  # tirn wall to good postion
    turn_to_angle(2)  # turn t 2 units
    gyro_time(35, 2, angle=0, kp=1.8)  # take 2 units
    gyro_follow(-35, 0.8, angle=2, kp=2, stop=True)  # backwored a bit
    no_wall_turn(90)  # turn to third unit
    gyro_follow(60, 11.5, angle=90, kp=2, stop=True)  # taking third unit

    # power to x
    turn_to_angle(-23)  # turn to power to x
    gyro_follow(-80, 11, angle=-36, kp=2, stop=True)  # go to power to x


def yellow_run():
    reset()
    wall_turn(90)
    gyro_follow(40, 18, 0, kp=0.4)  # drive until m12
    wall.run_angle(180, -180, then=Stop.HOLD, wait=True)
    wheels.drive(-70, -77)

    while True:
        pass


def haratza9():
    reset()
    wall_turn(42)
    gyro_time(450, 3, 0, kp=1.5)  # going to power plant
    gyro_follow(-300, 1, 0, kp=1.5)  # going to power plant
    gyro_time(400, 0.5, 0, kp=1.5)  # going to power plant

    wheels.drive(-400, -5)

    while True:
        pass


# ________________________________________________________________________________________________________________
# Utility Functions
# ________________________________________________________________________________________________________________


def breakpoint():
    pressed = hub.buttons.pressed()
    while not Button.CENTER in pressed:
        pressed = hub.buttons.pressed()


def reset():
    """Reset sensors and buttons
    Use at the beginning of each run
    """
    wall.reset_angle(0)
    hub.imu.reset_heading(0)


def turn_in_place(speed):
    """Turns in place by deg/s until stopped
    speed: deg/s of the robot"""
    leftW.run(speed * W_DISTANCE / W_DIAMETER)
    rightW.run(speed * W_DISTANCE / W_DIAMETER)


def gyro_until_black(base_speed, angle=0, kp=2, sensor=frontCS, stop=True, black=BLACK):
    """Follows a certain angle until sensor reaches reflected light threshold
    base_speed: mm/s"""

    base_speed = base_speed * 360 / W_CIRC

    while sensor.reflection() > black:
        # while sensor.color() != Color.BLACK:
        error = angle - hub.imu.heading()
        leftW.run(base_speed + int(error * kp))
        rightW.run(base_speed - int(error * kp))

    if stop:
        leftW.hold()
        rightW.hold()


def turn_until_black(speed, sensor=frontCS):
    """Turns in place until sensor reaches reflected light threshold
    speed: deg/s of the robot"""
    turn_in_place(speed)
    while sensor.reflection() > BLACK:
        pass

    leftW.hold()
    rightW.hold()


def gyro_follow(base_speed, distance, angle=0, kp=2, stop=True):
    """Drive straight on a certain angle for a set distance
    base_speed = mm/s
    distance == cm"""
    base_speed = base_speed * 360 / W_CIRC
    distance = cm_to_deg(distance)
    wheel_start = leftW.angle()

    while abs(distance) > abs(leftW.angle() - wheel_start):
        error = -int(hub.imu.heading() - angle)
        leftW.run(int(base_speed + error * kp))
        rightW.run(int(base_speed - error * kp))

    if stop:
        leftW.hold()
        rightW.hold()


def gyro_time(base_speed, seconds, angle=0, kp=2, stop=True):
    """Drive straight on a certain angle for a certain time
    base_speed = mm/s"""
    base_speed = base_speed * 360 / W_CIRC
    timer = StopWatch()
    timer.reset()

    while (timer.time()) < (seconds * 1000):
        error = -int(hub.imu.heading() - angle)
        leftW.run(int(base_speed + error * kp))
        rightW.run(int(base_speed - error * kp))

    if stop:
        leftW.hold()
        rightW.hold()


def turn_to_angle(angle, speed=180):
    """Turns to a specified absolute gyro angle"""

    timer = StopWatch()
    timer.reset()

    distance = angle - hub.imu.heading()
    robot_acceleration = wheels.settings()[3]
    wheels.settings(turn_rate=speed)
    wheels.turn(distance)
    wheels.settings(turn_rate=robot_acceleration)
    # direction = 1 if distance % 360 < 180 else -1

    # # Wait until the robot has turned the desired number of degrees
    # while abs(distance) > accuracy and (timer.time()) < (max_time * 1000):
    #     distance = abs(hub.imu.heading() - angle)
    #     speed = min(max_speed, max(distance + kp, min_speed))
    #     leftW.run(int(direction * speed))
    #     rightW.run(int(-direction * speed))
    #     # wait(20)

    leftW.hold()
    rightW.hold()


def wall_turn(angle, speed=180):
    """Turns the wall to a specified degree value
    speed: deg/s of the wall"""
    distance = angle - (wall.angle() % 360)
    wall.run_angle(speed, distance, then=Stop.HOLD, wait=True)


def no_wall_turn(angle, speed=180):
    """speed: deg/s"""
    robot_distance = angle - hub.imu.heading()
    direction = 1 if robot_distance % 360 < 180 else -1

    robot_acceleration = wheels.settings()[3]
    motor_acceleration = wall.control.limits()[1]
    wall.control.limits(acceleration=robot_acceleration)

    wheels.settings(turn_rate=speed)
    wall.run_angle(speed, -robot_distance, wait=False)
    wheels.turn((robot_distance), wait=True)

    wall.control.limits(acceleration=motor_acceleration)


def deg_to_cm(degrees):
    return (degrees / 360) * W_CIRC


def cm_to_deg(cm):
    return (cm * 10 / W_CIRC) * 360


def get_average_distance():
    """Returns the average distance of both wheels"""
    return deg_to_cm((abs(rightW.angle())) + abs(leftW.angle())) / 2


def follow_line(base_speed, distance, kp=3, side="right", sensor=frontCS, stop=True):
    """Follows a line with specified sensor for a set distance
    base_speed = mm/s"""
    base_speed = base_speed * 360 / W_CIRC
    wheels.drive(base_speed, 0)
    direction = 1 if base_speed > 0 else -1
    if side == "right":
        direction *= -1
    rightW.reset_angle(0)
    leftW.reset_angle(0)
    while abs(get_average_distance()) < abs(distance):
        error = TARGET - sensor.reflection()
        change = int(error * kp * direction)
        leftW.run(base_speed + change)
        rightW.run(base_speed - change)

    if stop:
        leftW.hold()
        rightW.hold()


def follow_line_time(
    base_speed, seconds, kp=3, side="right", sensor=frontCS, stop=True
):
    """Follows a line with specified sensor for a certain time
    base_speed = mm/s"""
    base_speed = base_speed * 360 / W_CIRC
    wheels.drive(base_speed, 0)
    direction = 1 if base_speed > 0 else -1
    if side == "left":
        direction *= -1
    rightW.reset_angle(0)
    leftW.reset_angle(0)
    timer = StopWatch()
    timer.reset()

    while (timer.time()) < (seconds * 1000):
        error = TARGET - sensor.reflection()
        change = int(error * kp * direction)
        leftW.run(base_speed + change)
        rightW.run(base_speed - change)

    if stop:
        leftW.hold()
        rightW.hold()


def linear_generator(duration):
    duration *= 10**3
    i: float = 0
    timer = StopWatch()
    while i < 1:
        yield i
        i = (timer.time()) / duration
    yield 1


def accelerate(seconds, start_power, end_power, stop=False):
    power_delta = end_power - start_power
    generator = linear_generator(seconds)

    for speed in generator:
        wheels.drive(int(start_power + (power_delta * speed)), 0)

    if stop:
        leftW.hold()
        rightW.hold()


# _______________________________________________________________________________________________________________
# Main Loop
# ________________________________________________________________________________________________________________

RUNS = [
    green_run,
    red_run,
    violet_run,
    blue_run,
    yellow_run,
    haratza9,
]  # Add every new run to this list!
RUN_COLORS = [
    Color.GREEN,
    Color.RED,
    Color.MAGENTA,
    Color.BLUE,
    Color.YELLOW,
    Color.BLACK,
]  # Match order with the previous list!


reset()


current_run = 0

while True:
    pressed = hub.buttons.pressed()
    if Button.LEFT in pressed or Button.RIGHT in pressed:
        try:
            current_run = RUN_COLORS.index(backCS.color())
            RUNS[current_run]()
            # hub.display.number(current_run)
        except ValueError as error:
            if str(error) != "object not in sequence":
                raise
