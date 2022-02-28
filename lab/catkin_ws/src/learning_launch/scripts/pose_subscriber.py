#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy as rp
from turtlesim.msg import Pose


def poss_callback(msg):
    rp.loginfo(f'turtle pose: x: {msg.x}, y: {msg.y}')


def pose_subscriber():
    rp.init_node('pose_subscriber', anonymous=True)
    rp.Subscriber('/turtle1/pose', Pose, poss_callback)
    rp.spin()


def main():
    pose_subscriber()


if __name__ == '__main__':
    main()
