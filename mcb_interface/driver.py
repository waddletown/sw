#!/usr/bin/env python3

from a_star import AStar
a_star = AStar()

heartbeat = False

while True:
    a_star.leds(heartbeat, not heartbeat, heartbeat)
    buttons = a_star.read_buttons()
    battery_millivolts = a_star.read_battery_millivolts()
    encoders = a_star.read_encoders()
    print encoders
