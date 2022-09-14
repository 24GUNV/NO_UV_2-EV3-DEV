from pybricks.ev3devices import (ColorSensor, GyroSensor, InfraredSensor,
                                 Motor, TouchSensor, UltrasonicSensor)
from pybricks.hubs import EV3Brick
from pybricks.media.ev3dev import ImageFile, SoundFile
from pybricks.messaging import BluetoothMailboxServer, Mailbox
from pybricks.parameters import Button, Color, Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import DataLog, StopWatch, wait

# Importing other modules
from turn import *

# # Initialize the brick
ev3 = EV3Brick()

# Initializing the turning functions


left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
# arm_rotator = Motor(Port.C)
# arm_claw = Motor(Port.B)

turn = Turning(left_motor, right_motor)

turn.SmoothStart_Left(100, 100, 30, 200)

# # Initialize the color sensor.
# front_light = ColorSensor(Port.S1)
# y_axis_ultrasonic = UltrasonicSensor(Port.S2)
# gyroscope = GyroSensor(Port.S3)
# light_sensor = ColorSensor(Port.S4)

