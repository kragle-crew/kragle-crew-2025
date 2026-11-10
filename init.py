from pybricks.hubs import PrimeHub
from pybricks.robotics import DriveBase
from pybricks.pupdevices import Motor, ForceSensor, ColorSensor 
from pybricks.parameters import Port, Direction, Axis

# The hub is mounted with its FRONT side facing up, so tell the gyro.
# (checked with check_orientation.py). front_side still needs checking:
# it only changes pitch/roll, not heading.
hub = PrimeHub(top_side=Axis.X, front_side=Axis.Z)
left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B, Direction.CLOCKWISE)
WHEEL_DIAMETER = 57.5  # mm
AXLE_TRACK = 100       # mm, distance between the two wheels
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)
lift = Motor(Port.D)
pressure = ForceSensor(Port.C)
colorLeft = ColorSensor(Port.E)
colorRight = ColorSensor(Port.F)

print("init complete")