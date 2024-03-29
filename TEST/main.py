#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (GyroSensor, InfraredSensor,
                                 Motor, TouchSensor, UltrasonicSensor)
from pybricks.media.ev3dev import ImageFile, SoundFile
from pybricks.messaging import BluetoothMailboxServer, Mailbox
from pybricks.parameters import Button, Color, Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import DataLog, StopWatch, wait

# Importing other modules
from turn import *
from CurrentPower import CurrentPower
from LineFollow import LineFollow
from utils import *


# Initialize the brick
ev3 = EV3Brick()

# Initialize the motors
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)

# Initializing the sensors
# left_sensor = ColorSensor(Port.S1)
# right_sensor = ColorSensor(Port.S2)


# Initialize class to store current power to run motors
CurrentPower = CurrentPower()

# Initializing the turning class
turn = Turning(left_motor, right_motor, CurrentPower)

# Initializing the line following class
# line_follow = LineFollow(left_sensor, right_sensor, left_motor, right_motor, CurrentPower)
# line_follow.SET_DEFAULT()

turn.SmoothStart_Left(100, 100, 30, 200)
turn.SmoothStart_Right(100, 100, 30, 200)
