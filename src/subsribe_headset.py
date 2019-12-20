#!/usr/bin/env python

import rospy
import serial
from geometry_msgs.msg import PoseStamped
import tf.transformations
from math import pi

com = serial.Serial('/dev/ttyACM0', 57600, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)

def rad2byte(i):
    i = int(i*180.0/pi)
    if i > 180:
        i = i-360
    if i < -180:
        i = 360+i
    if i >= 0:
        return i
    else:
        return 256+i

def callback(pose):
    quaternion = (pose.pose.orientation.x, pose.pose.orientation.y,
                  pose.pose.orientation.z, pose.pose.orientation.w)
    # rospy.loginfo("Angle1: %d, Speed1: %d, Angle2: %d, Speed2: %d" % (data.angle1,data.speed1,data.angle2,data.speed2))
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
    print str(roll) + ', ' + str(pitch) + ',' + str(yaw)

    values = bytearray([0x80, rad2byte(yaw), 30, rad2byte(pitch), 30, 0x7f])
    print [hex(i) for i in values]
    # print values
    com.write(values)
    #data = com.read(2)
    #print data

def listener():
    rospy.init_node('subscribe_headset_data', anonymous=True)
    # rospy.Subscriber('publish_headset_data', HeadMotion, callback)
    rospy.Subscriber('/vive/twist5', PoseStamped, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

