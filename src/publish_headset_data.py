#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import tf.transformations
import socket
from math import pi

RATE = 20

# degree to radians
def d2r(i):
    return float(i) * pi / 180.0

def fillin_pos(data):
    qt = tf.transformations.quaternion_from_euler(d2r(data[3]), d2r(data[4]), d2r(data[5]))
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "world"
    pose.pose.position.x = float(data[0])
    pose.pose.position.y = float(data[1])
    pose.pose.position.z = float(data[2])
    pose.pose.orientation.x = qt[0]
    pose.pose.orientation.y = qt[1]
    pose.pose.orientation.z = qt[2]
    pose.pose.orientation.w = qt[3]

    return pose

def vive_status_pub():
    pub_right = rospy.Publisher('/vive/twist3', PoseStamped, queue_size=10)
    pub_left  = rospy.Publisher('/vive/twist4', PoseStamped, queue_size=10)
    pub_head  = rospy.Publisher('/vive/twist5', PoseStamped, queue_size=10)

    rospy.init_node('vive_status', anonymous=True)
    rate = rospy.Rate(RATE)
    while not rospy.is_shutdown():
        buffer, addr = sock.recvfrom(2048)
        buffer = buffer.split(',')

        head_pos = fillin_pos(buffer[1:7])
        right_pos = fillin_pos(buffer[8:14])
        left_pos = fillin_pos(buffer[15:26])
        
        pub_right.publish(right_pos)
        pub_left.publish(left_pos)
        pub_head.publish(head_pos)

        rate.sleep()

if __name__ == '__main__':
    address = ("130.215.206.220", 8001)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto('request', address)
    try:
        vive_status_pub()
    except rospy.ROSInterruptException:
        pass
