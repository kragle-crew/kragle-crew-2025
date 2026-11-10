# Finds out how the hub is mounted on the robot, so the gyro measures the
# right direction. Run it with:  pybricksdev run ble check_orientation.py
#
# 1. Set the robot flat on the table.     The side it prints is the TOP side.
# 2. Lift the robot so its FRONT points   The side it prints is the FRONT side.
#    straight up at the ceiling.
#
# Press the CENTER button to stop.

from pybricks.hubs import PrimeHub
from pybricks.tools import wait

hub = PrimeHub()

while True:
    print("side of the hub facing up:", hub.imu.up())
    wait(1000)
