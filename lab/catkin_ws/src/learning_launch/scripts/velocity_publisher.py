#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy as rp
from geometry_msgs.msg import Twist


def velocity_publisher():
    rp.init_node('velocity_publisher', anonymous=True)
    turtle_vel_pub = rp.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    pub_freq = rp.get_param('~publish_frequency')
    rate = rp.Rate(pub_freq)
    while not rp.is_shutdown():
        vel_msg = Twist()
        vel_msg.linear.x = rp.get_param('~linear_x')
        vel_msg.angular.z = rp.get_param('~group/angular_z')

        turtle_vel_pub.publish(vel_msg)
        rp.loginfo(f'pub turtle velocity cmd: {vel_msg.linear.x} m/s, {vel_msg.angular.z} rad/s')
        rate.sleep()


def main():
    try:
        velocity_publisher()
    except rp.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
