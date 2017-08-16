#!/usr/bin/env python3

from a_star import AStar
a_star = AStar()
from time import sleep
from math import pi

heartbeat = 0
wheel_diameter = 0.07
wheelbase = 0.141
ticks_per_output_revolution = 12*120
ticks_per_meter = ticks_per_output_revolution/(pi*wheel_diameter)
meters_per_tick = 1.0/ticks_per_meter

old_x_dist = 0
old_angle = 0

while True:
    blink = heartbeat % 50 <= 25
    a_star.leds(0, blink, not blink)
    buttons = a_star.read_buttons()
    battery_millivolts = a_star.read_battery_millivolts()
    encoders = a_star.read_encoders()
    left_dist, right_dist = (encoder*meters_per_tick for encoder in encoders)
    x_dist = (left_dist + right_dist)/2.0
    angle = (right_dist-left_dist)/wheelbase
    x_dot = (x_dist - old_x_dist)/0.01
    angle_dot = (angle - old_angle)/0.01
    old_x_dist = x_dist
    old_angle = angle
    #print("Left Distance: ", left_dist, "\tRight Distance: ", right_dist)
    print("Forward Distance: ", x_dist, "\tAngle: ", angle)
    print("xdot: ", x_dot, "\tangle_dot: ", angle_dot)
    heartbeat += 1
    sleep(0.01)
