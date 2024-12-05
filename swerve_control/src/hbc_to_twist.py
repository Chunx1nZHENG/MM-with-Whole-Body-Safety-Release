#!/usr/bin/env python3
import rospy
from joy_manager_msgs.msg import AnyJoy
from geometry_msgs.msg import Twist

class Converter:
    def __init__(self):

        self.linear_scaling = rospy.get_param("~linear_scaling", 1.0)
        self.angular_scaling = rospy.get_param("~angular_scaling", 1.0)

        self.subscribe = rospy.Subscriber("/anyjoy/operator", AnyJoy, self.callback, queue_size=5)
        self.publisher = rospy.Publisher("/hbc_twist", Twist, queue_size=1)

    def callback(self, any_joy : AnyJoy):
        twist = Twist()
        twist.angular.z = any_joy.joy.axes[3] * self.angular_scaling
        twist.linear.x = any_joy.joy.axes[1] * self.linear_scaling
        twist.linear.y = any_joy.joy.axes[0] * self.linear_scaling
        self.publisher.publish(twist)

if __name__ == '__main__':
    rospy.init_node('hbc_to_twist')
    converter = Converter()
    rospy.spin() 