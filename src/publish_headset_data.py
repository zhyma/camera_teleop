#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import tf.transformations
import socket
from math import pi

RATE = 20

def get_angle(i):
    i = int(float(i))
    if i > 180:
        i = i-360
    if i < -180:
        i = 360+i
    return i

def d2r(i):
    return float(i) * pi / 180.0

def vive_status_pub():
    pub_headquat = rospy.Publisher('/vive/twist5', PoseStamped, queue_size=10)
    rospy.init_node('vive_status', anonymous=True)
    rate = rospy.Rate(RATE)
    while not rospy.is_shutdown():
        buffer, addr = sock.recvfrom(2048)
        buffer = buffer.split(',')

        head_pos = PoseStamped()
        qt = tf.transformations.quaternion_from_euler(d2r(buffer[4]), d2r(buffer[5]), d2r(buffer[6]))
        head_pos.header.stamp = rospy.Time.now()
        head_pos.header.frame_id = "map"
        head_pos.pose.position.x = float(buffer[1])
        head_pos.pose.position.y = float(buffer[2])
        head_pos.pose.position.z = float(buffer[3])
        head_pos.pose.orientation.x = qt[0]
        head_pos.pose.orientation.y = qt[1]
        head_pos.pose.orientation.z = qt[2]
        head_pos.pose.orientation.w = qt[3]
        pub_headquat.publish(head_pos)

        rate.sleep()

if __name__ == '__main__':
    address = ("130.215.206.220", 8001)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto('request', address)
    try:
        vive_status_pub()
    except rospy.ROSInterruptException:
        pass
