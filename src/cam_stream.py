import threading
import cv2
import os
import numpy as np

import rospy
from std_msgs.msg import String, Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class all_cams():
    head, right, left = range(3)


class videoThread(threading.Thread):
    def __init__(self, threadID, name, ip_addr):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.t = 0
        self.width = 640
        self.height = 480
        self.curr_cam = 'head ctrl head cam'
        self.curr_state = 'pause'
        self.bridge = CvBridge()
        self.head_img = np.zeros((self.height, self.width, 3), np.uint8)
        self.right_img = np.zeros((self.height, self.width, 3), np.uint8)
        self.left_img = np.zeros((self.height, self.width, 3), np.uint8)

        rospy.Subscriber('/cam_select', String, self.callback_cams, queue_size = 1)
        rospy.Subscriber('/vive_ctrl_status', String, self.callback_state, queue_size = 1)
        rospy.Subscriber('/rs_head/image_raw', Image, self.callback_cam_h, queue_size = 1)
        rospy.Subscriber('/rs_right/image_raw', Image, self.callback_cam_r, queue_size = 1)
        rospy.Subscriber('/rs_left/image_raw', Image, self.callback_cam_l, queue_size = 1)
        rospy.init_node('cam_stream', anonymous=True)

        self.out_send = cv2.VideoWriter(
        'appsrc ! videoconvert ! \
             tee name="local" ! queue ! jpegenc! \
                rtpjpegpay ! udpsink host=' + ip_addr +' port=5000 sync=false local. ! queue ! autovideosink',
        cv2.CAP_GSTREAMER, 0, 25, (self.width, self.height), True)

        if not self.out_send.isOpened():
            print('VideoWriter not opened')
            exit(0)

        self.running = True

    def callback_cams(self, data):
        self.curr_cam = data.data
        
    def callback_state(self, data):
        self.curr_state = data.data

    def callback_cam_h(self, data):
        try:
            self.head_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print (e)

    def callback_cam_r(self, data):
        try:
            self.right_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print (e)

    def callback_cam_l(self, data):
        try:
            self.left_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print (e)

    def run(self):
        y_offset = 10
        x_offset = 430
        while self.running:
            if self.curr_cam == 'h2r':
                res = cv2.resize(self.head_img, (200, 150))
                frame = self.right_img.copy()
                #frame[y_offset:y_offset+ res.shape[0], x_offset:x_offset+res.shape[1]] = res
                ctrl_text = 'Head ctrl right cam'
            elif self.curr_cam == 'r2r':
                res = cv2.resize(self.head_img, (200, 150))
                frame = self.right_img.copy()
                #frame[y_offset:y_offset+ res.shape[0], x_offset:x_offset+res.shape[1]] = res
                ctrl_text = 'Rhand ctrl right cam'
            elif self.curr_cam == 'h2l':
                res = cv2.resize(self.head_img, (200, 150))
                frame = self.left_img.copy()
                #frame[y_offset:y_offset+ res.shape[0], x_offset:x_offset+res.shape[1]] = res
                ctrl_text = 'Head ctrl left cam'
            elif self.curr_cam == 'l2l':
                res = cv2.resize(self.head_img, (200, 150))
                frame = self.left_img.copy()
                #frame[y_offset:y_offset+ res.shape[0], x_offset:x_offset+res.shape[1]] = res
                ctrl_text = 'LHand ctrl left cam'
            else:
                frame = self.head_img.copy()
                ctrl_text = 'Head ctrl head cam'
                
            frame = cv2.putText(frame, self.curr_state, (50, 50), cv2.FONT_HERSHEY_SIMPLEX,
                                    1, (255, 255, 255), 4, cv2.LINE_AA)
            frame = cv2.putText(frame, self.curr_state, (50, 50), cv2.FONT_HERSHEY_SIMPLEX,
                                    1, (0, 0, 0), 2, cv2.LINE_AA)
            frame = cv2.putText(frame, ctrl_text, (50, 100), cv2.FONT_HERSHEY_SIMPLEX,
                                    1, (255, 255, 255), 4, cv2.LINE_AA)
            frame = cv2.putText(frame, ctrl_text, (50, 100), cv2.FONT_HERSHEY_SIMPLEX,
                                    1, (0, 0, 0), 2, cv2.LINE_AA)
            self.out_send.write(frame)
            #cv2.imshow('send', frame)

            key = cv2.waitKey(1)
            if key & 0xFF == ord('q'):
                break
        self.running = False
        self.out_send.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    ip_addr = '130.215.206.121'

    v_thread = videoThread(0, 'videoT', ip_addr)
    v_thread.start()
