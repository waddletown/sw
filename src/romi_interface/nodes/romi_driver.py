#!/usr/bin/env python

from romi_interface.romi import Romi
import rospy
import atexit
import time
from math import pi, sqrt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

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
        #self.romi.reset_odometry()

    def cmd_vel_callback(self, twist):
        #print twist
        vx     = constrain(twist.linear.x, -1*self.max_vx, self.max_vx)
        vtheta = constrain(twist.angular.z, -1*self.max_vtheta, self.max_vtheta)
        self.romi.velocity_command(vx, vtheta)

    def shutdown(self):
        self.romi.velocity_command(0, 0)
        self.romi.reset_odometry()

    def run(self):
        print "Romi Running"
        while not rospy.is_shutdown():
            millivolts = self.romi.read_battery_millivolts()[0]
            v_x, v_theta, x, y, theta = self.romi.read_odometry()

            batt_msg = Float32()
            batt_msg.data = millivolts/1000.

            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = "odom"
            odom_msg.pose.pose.position.x     = x
            odom_msg.pose.pose.position.y     = y
            odom_msg.pose.pose.orientation.z = theta
            odom_msg.pose.pose.orientation.w = 1-sqrt(theta**2)
            odom_msg.twist.twist.linear.x = v_x
            odom_msg.twist.twist.angular.z = v_theta

            self.odom_pub.publish(odom_msg)
            self.batt_pub.publish(batt_msg)

            self.loop_rate.sleep()


if __name__=="__main__":
  mcb = RomiMCBDriver()
  mcb.run()
