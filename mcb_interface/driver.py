#!/usr/bin/env python3

from romi import Romi
romi = Romi()
from time import sleep
# from math import pi


while True:
    battery_millivolts = romi.read_battery_millivolts()
    v_x, v_theta, x, y, theta = romi.read_odometry()

    print "Battery Voltage: " + battery_millivolts*1000 + " Volts."
    print "Vx: " + v_x + " m/s"
    print "Vtheta: " + v_theta + "rad/s"

    sleep(0.01)
