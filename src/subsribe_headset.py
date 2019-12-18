#!/usr/bin/env python

import rospy
import serial
from camera_teleop.msg import HeadMotion

com = serial.Serial('/dev/ttyACM0', 57600, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)

def int2byte(i):
    if i > 180:
        i = i-360
    if i < -180:
        i = 360+i
    if i >= 0:
        return i
    else:
        return 256+i

def callback(data):
    rospy.loginfo("Angle1: %d, Speed1: %d, Angle2: %d, Speed2: %d" % (data.angle1,data.speed1,data.angle2,data.speed2))
    
    values = bytearray([0x80, int2byte(data.angle1), data.speed1, int2byte(data.angle2), data.speed2, 0x7f])
    print [hex(i) for i in values]
    # print values
    com.write(values)
    #data = com.read(2)
    #print data

def listener():
    rospy.init_node('subscribe_headset_data', anonymous=True)
    rospy.Subscriber('publish_headset_data', HeadMotion, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

