#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Joy
import tf.transformations
import socket
from math import pi

RATE = 20

# degree to radians
def d2r(i):
    return float(i) * pi / 180.0

def fillin_pos(data, rot):
    #rot is the offset of orientation for given frame
    qt = tf.transformations.quaternion_from_euler(d2r(data[3])+rot[0],
                                                  d2r(data[4])+rot[1], d2r(data[5])+rot[2])
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

    msg = TransformStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "world"
    msg.child_frame_id = "test"
    msg.transform.translation.x = float(data[0])
    msg.transform.translation.y = float(data[1])
    msg.transform.translation.z = float(data[2])
    msg.transform.rotation.x = qt[0]
    msg.transform.rotation.y = qt[1]
    msg.transform.rotation.z = qt[2]
    msg.transform.rotation.w = qt[3]

    return pose, msg

def fillin_joy(data, ctrl_name):
    msg = Joy()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = ctrl_name
    msg.axes =  [float(i) for i in data[0:3]]
    msg.buttons = [int(i) for i in data[3:7]]
    return msg


def vive_status_pub():
    # Don't know what is the /vive/twist3 or 4 is for.
    pub_twist_r = rospy.Publisher('/vive/twist3', PoseStamped, queue_size=10)
    pub_twist_l = rospy.Publisher('/vive/twist4', PoseStamped, queue_size=10)

    # This one is send to control the head camera
    pub_head    = rospy.Publisher('/vive/twist5', PoseStamped, queue_size=10)
    
    # Those are send to control the end effectors
    pub_pos_r = rospy.Publisher('/Right_Hand',TransformStamped, queue_size=10)
    pub_pos_l = rospy.Publisher('/Left_Hand', TransformStamped, queue_size=10)

    pub_key_r = rospy.Publisher('/vive/controller_LHR_FFFB7FC3/joy', Joy, queue_size=10)
    pub_key_l = rospy.Publisher('/vive/controller_LHR_FF7FBBC0/joy', Joy, queue_size=10)

    rospy.init_node('vive_status', anonymous=True)
    rate = rospy.Rate(RATE)
    while not rospy.is_shutdown():
        try:
       	    sock.settimeout(1)
            buffer, addr = sock.recvfrom(2048)
            buffer = buffer.split(',')
            print buffer

            head_pos, _= fillin_pos(buffer[1:7], [0, 0, 0])
            right_pos, msg_r = fillin_pos(buffer[8:14], [-pi/2, pi, 0])
            left_pos, msg_l  = fillin_pos(buffer[15:21], [pi/2, pi, 0])

            joy_r = fillin_joy(buffer[22:29], 'controller_LHR_FFFB7FC3')
            joy_l = fillin_joy(buffer[30:37], 'controller_LHR_FF7FBBC0')

            pub_twist_r.publish(right_pos)
            pub_pos_r.publish(msg_r)
            pub_twist_l.publish(left_pos)
            pub_pos_l.publish(msg_l)
            pub_head.publish(head_pos)
            pub_key_r.publish(joy_r)
            pub_key_l.publish(joy_l)

            rate.sleep()
        except:
            print 'no server detected, reconnecting'
            sock.sendto('request', address)
            rate.sleep()

if __name__ == '__main__':
    address = ("130.215.206.220", 8001)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto('request', address)
    try:
        vive_status_pub()
    except rospy.ROSInterruptException:
        pass
