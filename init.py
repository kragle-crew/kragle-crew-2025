from pybricks.hubs import PrimeHub
from pybricks.robotics import DriveBase
from pybricks.pupdevices import Motor, ForceSensor, ColorSensor 
from pybricks.parameters import Port, Direction

hub = PrimeHub()
left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B, Direction.CLOCKWISE)
robot = DriveBase(left_motor, right_motor, 57.5, 100)
lift = Motor(Port.D)
pressure = ForceSensor(Port.C)
colorLeft = ColorSensor(Port.E)
colorRight = ColorSensor(Port.F)

print("init complete")