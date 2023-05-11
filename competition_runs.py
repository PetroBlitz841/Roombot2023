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
    Color.VIOLET,
    Color(h=339, s=85, v=94),
]

frontCS.detectable_colors(COLOR_LIST)
backCS.detectable_colors(COLOR_LIST)

TARGET = 60
BLACK = 20
WHITE = 98

# Reset Wheels
leftW.reset_angle(0)
rightW.reset_angle(0)
settings = wheels.settings()

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
print("is ready: ", hub.imu.ready())


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
    wheels.settings(straight_speed=400)
    wheels.straight(90, then=Stop.NONE, wait=True)
    accelerate(2, 400, 100, stop=True)  # slows before tv
    wheels.straight(-150)
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
    wheels.straight(-270, then=Stop.HOLD, wait=True)  # back from wind turbine
    wheels.straight(40)
    turn_to_angle(115)  # turn towards hybrid car
    wall_turn(0)
    gyro_until_black(-250, 115, sensor=backCS, stop=False)
    wheels.straight(-150)
    arm.run_angle(900, 120, then=Stop.HOLD, wait=True)  # reset arm
    turn_to_angle(155)
    wheels.straight(-100)
    arm.run_angle(1000, -150, then=Stop.HOLD, wait=True)  # do mission
    arm.run_angle(900, 130, then=Stop.HOLD, wait=True)

    # HOMEWARDS
    wheels.settings(straight_speed=500)
    turn_to_angle(130)
    wheels.straight(300)
    wheels.drive(500, 30)


def red_run():
    reset()
    wheels.settings(straight_speed=400)

    # TOY FACTORY
    wheels.straight(450)

    # POWER PLANT
    wheels.straight(-230)  # back from toy factory
    no_wall_turn(-55)
    wall_turn(90)
    wheels.drive(350, 0)
    wait(1000)
    wheels_stop()
    turn_to_angle(-29)
    gyro_until_black(300, -29, stop=False)  # until black
    wheels.straight(55)
    no_wall_turn(46)
    wheels.straight(1000, wait=False)
    while not frontCS.color() == Color.GREEN:
        ...
    wheels_stop()
    wall_turn(90)  # set wall
    follow_line(-170, 170, kp=3, sensor=backCS, stop=False)
    wheels.drive(-300, 0)
    wait(800)
    wheels_stop()
    if hub.imu.heading() >= 50:
        rightW.run_angle(20, 35)

    # SMART GRID
    wheels.settings(straight_speed=200)
    wheels.straight(50)
    wheels.settings(straight_speed=200)
    turn_to_angle(46)
    wheels.straight(750)

    # WATER UNITS
    wheels.straight(-85)
    turn_to_angle(-65)
    wheels.settings(straight_speed=300)
    wheels.straight(250)
    wheels.settings(straight_speed=200)

    # OIL PLATFORM
    wheels.straight(-130)
    turn_to_angle(-25)
    gyro_until_black(300, -25, stop=False)  # towards line
    wheels.straight(70)
    turn_until_black(-150)
    rightW.run_angle(40, 15)
    follow_line(150, distance=200, kp=3.4, side="right")
    wall_turn(45)
    follow_line(150, 130, kp=3.4, side="right")
    wheels.settings(straight_speed=350)
    wheels.straight(270)
    for i in range(3):
        wheels.straight(-80)
        wheels.straight(170)

    # HOMEWARDS
    wheels.straight(-120)
    no_wall_turn(-125)
    wheels.settings(straight_speed=500)
    wheels.straight(70)
    wall_turn(90)
    wheels.straight(700)
    wheels.turn(20)
    wheels.straight(10000)


def magenta_run():
    reset()
    wheels.settings(straight_speed=500)
    wheels.straight(230)
    wall_turn(-90)  # turn wall to move enrgay unite
    wheels.drive(500, 0)  # take unite and water unite
    wait(800)  # wait fir water to drop
    wheels.drive(-600, 0)


def blue_run():
    reset()

    # ENERGY STORAGE
    gyro_until_black(250, angle=0, kp=2, sensor=frontCS, stop=False)
    wheels.straight(90)
    wall.run_angle(600, 45, then=Stop.HOLD, wait=True)
    turn_to_angle(38)
    follow_line_time(170, 2.8, kp=3, side="left", sensor=frontCS, stop=True)

    # SOLAR FARM
    wheels.settings(straight_speed=250)
    wheels.straight(-30)
    turn_to_angle(90)  # turn
    wheels.straight(190)
    wall.run_angle(180, -135, then=Stop.HOLD, wait=True)  # tirn wall to good postion
    turn_to_angle(0)  # turn t 2 units
    wheels.drive(250, 0)  # take 2 units
    wait(1000)
    wheels.straight(-20)
    no_wall_turn(90)  # turn to third unit
    wheels.settings(straight_speed=500)
    wheels.straight(360)

    # POWERTO X
    turn_to_angle(-35)  # turn to power to x
    wheels.settings(straight_speed=300)
    wheels.straight(-385)


def yellow_run():
    reset()
    wall_turn(90)
    wheels.straight(600)
    # gyro_follow(200, 60, 0, kp=0.2)  # drive until m12
    wall.run_angle(1000, -180, then=Stop.HOLD, wait=True)
    wheels.drive(-450, 15)


def haratza9():
    reset()
    wall.run_angle(700, 45)
    wheels.drive(450, 0)
    wait(2000)
    wall.run_angle(700, 45)
    wall.run_angle(700, -45)
    wheels.drive(-400, 0)


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
    wheels.settings(settings[0], settings[1], settings[2], settings[3])
    if not hub.imu.ready():
        hub.display.icon(
            [
                [100, 0, 0, 0, 100],
                [0, 100, 0, 100, 0],
                [0, 0, 100, 0, 0],
                [0, 100, 0, 100, 0],
                [100, 0, 0, 0, 100],
            ]
        )
        wait(500)


def deg_to_cm(degrees):
    return (degrees / 360) * W_CIRC


def cm_to_deg(cm):
    return (cm * 10 / W_CIRC) * 360


def turn_in_place(speed):
    """Turns in place by deg/s until stopped
    speed: deg/s of the robot"""
    leftW.run(speed * W_DISTANCE / W_DIAMETER)
    rightW.run(-speed * W_DISTANCE / W_DIAMETER)


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


def drive_until_black(speed, sensor=frontCS, turn_rate=0, stop=True, black=BLACK):
    while sensor.reflection() > black:
        wheels.drive(speed, turn_rate=turn_rate)

    if stop:
        wheels_stop()


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


def turn_to_angle(angle, speed=200, max_time=3):
    """Turns to a specified absolute gyro angle"""

    timer = StopWatch()
    timer.reset()

    distance = angle - hub.imu.heading()
    robot_acceleration = wheels.settings()[3]
    wheels.settings(turn_rate=speed)
    wheels.turn(distance)

    # while (timer.time()) < (max_time * 1000) and angle - hub.imu.heading() > 1:
    #     ...
    wheels.settings(turn_rate=robot_acceleration)


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


def get_average_distance():
    """Returns the average distance of both wheels"""
    return deg_to_cm((abs(rightW.angle())) + abs(leftW.angle())) / 2


def wheels_stop():
    rightW.hold()
    leftW.hold()


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
        wheels_stop()


def follow_line_until_color(
    base_speed, color=Color.GREEN, kp=3, side="right", sensor=frontCS, stop=True
):
    """Follows a line with specified sensor until the sensor sees a certain color
    base_speed = mm/s"""
    base_speed = base_speed * 360 / W_CIRC
    wheels.drive(base_speed, 0)
    direction = 1 if base_speed > 0 else -1
    if side == "right":
        direction *= -1

    while frontCS.color() != color:
        error = TARGET - sensor.reflection()
        change = int(error * kp * direction)
        leftW.run(base_speed + change)
        rightW.run(base_speed - change)

    if stop:
        wheels_stop()


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
        wheels_stop()


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

GREEN = 0
RED = 0
MAGENTA = (339, 85, 94)
BLUE = 0
YELLOW = 0
BLACK = 0

RUNS = [
    green_run,
    red_run,
    magenta_run,
    blue_run,
    yellow_run,
    haratza9,
]  # Add every new run to this list!
RUN_COLORS = [
    Color.GREEN,
    Color.RED,
    Color(h=339, s=85, v=94),
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
