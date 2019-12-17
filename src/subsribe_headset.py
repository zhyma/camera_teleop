#!/usr/bin/env python

import rospy
import serial
from camera_teleop.msg import HeadMotion

def callback(data):
   rospy.loginfo("StartBit: %s, Angle1: %d, Speed1: %d, Angle2: %d, Speed2: %d, Stop Bit: %s" % (data.startBit,data.angle1,data.speed1,data.angle2,data.speed2,data.stopBit))
   pass
   com = serial.Serial('/dev/ttyACM0', 57600, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
   pass
   values = bytearray([data.startBit, data.angle1, data.speed1, data.angle2, data.speed2, data.stopBit])
   # print values
   com.write(values)
   data = com.read(2)
   print data

def listener():
   rospy.init_node('subscribe_headset_data', anonymous=True)
   rospy.Subscriber("publish_headset_data", HeadMotion, callback)

   rospy.spin()

if __name__ == '__main__':
   listener()

