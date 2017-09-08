#!/usr/bin/env python3

from romi import Romi
romi = Romi()
from time import sleep
import atexit
import time
from math import pi

start_time = time.time()

romi.reset_odometry()

def at_exit(romi):
    romi.velocity_command(0, 0)
    print time.time() - start_time

atexit.register(at_exit, romi = romi)

while True:
    battery_millivolts = romi.read_battery_millivolts()
    v_x, v_theta, x, y, theta = romi.read_odometry()
    romi.velocity_command(0.0, pi*100)

    print "Battery Voltage: ", battery_millivolts[0], " Volts."
    print "Vx: ", v_x, " m/s"
    print "Vtheta: ", v_theta, "rad/s"
    print "X: ", x, " Y: ", y, " Theta: ", theta

    sleep(0.01)
