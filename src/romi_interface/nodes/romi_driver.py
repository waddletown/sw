#!/usr/bin/env python

from romi_interface.romi import Romi
import atexit
import time
from math import pi, sqrt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

# start_time = time.time()
#
# romi = Romi()
# romi.reset_odometry()

def constrain(val, min_val, max_vel):
    return min(max_val, max(min_val, val))

# def at_exit(romi):
#     romi.velocity_command(0, 0)
#     print time.time() - start_time
#
# atexit.register(at_exit, romi = romi)
#
# while True:
#     battery_millivolts = romi.read_battery_millivolts()
#     v_x, v_theta, x, y, theta = romi.read_odometry()
#     romi.velocity_command(0.0, pi*100)
#
#     print "Battery Voltage: ", battery_millivolts[0], " Volts."
#     print "Vx: ", v_x, " m/s"
#     print "Vtheta: ", v_theta, "rad/s"
#     print "X: ", x, " Y: ", y, " Theta: ", theta
#
#     time.sleep(0.01)


class RomiMCBDriver(object):
    def __init__(self):
        self.max_vx = 0.5
        self.max_vtheta = 2*pi

        rospy.init_node("romi_driver")
        rospy.on_shutdown(self.shutdown)

        self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size = 10)
        self.batt_pub = rospy.Publisher("battery_voltage",  Float32, queue_size = 10)

        self.loop_rate = rospy.Rate(20)
        self.romi = Romi()
        self.romi.reset_odometry()

    def cmd_vel_callback(self, twist):
        vx     = constrain(twist.linear.x, -1*self.max_vx, self.max_vx)
        vtheta = constrain(twist.angular.z, -1*self.max_vtheta, self.max_vtheta)
        romi.velocity_command(vx, vtheta)

    def shutdown(self):
        romi.velocity_command(0, 0)
        romi.reset_odometry()

    def run(self):
        while not rospy.is_shutdown():
            millivolts = romi.read_battery_millivolts()
            v_x, v_theta, x, y, theta = romi.read_odometry()

            batt_msg = Float32()
            batt_msg.data = millivolts/1000.

            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.now()
            odom_msg.header.frame_id = "odom"
            odom_msg.pose.pose.position.x     = x
            odom_msg.pose.pose.position.y     = y
            odom_msgs.pose.pose.orientation.z = theta
            odom_msgs.pose.pose.orientation.w = 1-sqrt(theta)
            odom_msgs.twist.linear.x = v_x
            odom_msgs.twist.angular.z = v_theta

            self.odom_pub.publish(odom_msg)
            self.batt_pub.publish(batt_msg)

            self.loop_rate.sleep()
