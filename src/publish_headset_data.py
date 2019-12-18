#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from camera_teleop.msg import HeadMotion
import socket

RATE = 20

def get_angle(i):
    i = int(float(i))
    if i > 180:
        i = i-360
    if i < -180:
        i = 360+i
    return i

def vive_status_pub():
    pub = rospy.Publisher('publish_headset_data', HeadMotion, queue_size=10)
    rospy.init_node('vive_status', anonymous=True)
    rate = rospy.Rate(RATE)
    while not rospy.is_shutdown():
        buffer, addr = sock.recvfrom(2048)
        buffer = buffer.split(',')
        rospy.loginfo(buffer)
        head_motion = HeadMotion()
        head_motion.angle1 = get_angle(buffer[4])
        head_motion.speed1 = 30
        head_motion.angle2 = get_angle(buffer[6])
        head_motion.speed2 = 30
        rospy.loginfo(head_motion)
        pub.publish(head_motion)
        rate.sleep()

if __name__ == '__main__':
    address = ("130.215.206.220", 8001)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto('request', address)
    try:
        vive_status_pub()
    except rospy.ROSInterruptException:
        pass
