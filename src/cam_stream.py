import threading
import cv2
import os
import numpy as np

import rospy
from std_msgs.msg import String, Int8

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
        self.curr_cam = 'head_cam'

        # self.cam_selection = rospy.Subscriber('/cam_select', String, self.callback_cams, queue_size = 1)
        # self.cam_head = rospy.Subscriber('/rs/color/image/raw', String, self.callback_cam_h, queue_size = 1)
        # self.cam_right = rospy.Subscriber('/rs/color/image/raw', String, self.callback_cam_r, queue_size = 1)
        # self.cam_left = rospy.Subscriber('/rs/color/image/raw', String, self.callback_cam_l, queue_size = 1)

        rospy.Subscriber('/cam_select', String, self.callback_cams, queue_size = 1)
        rospy.Subscriber('/rs/color/image/raw', String, self.callback_cam_h, queue_size = 1)
        rospy.Subscriber('/rs/color/image/raw', String, self.callback_cam_r, queue_size = 1)
        rospy.Subscriber('/rs/color/image/raw', String, self.callback_cam_l, queue_size = 1)

        # self.cap_pana = cv2.VideoCapture(
        #     'v4l2src device=' + dev_pana + ' ! image/jpeg,width=1920,height=1080,framerate=30/1 ! decodebin ! videoconvert ! appsink')
        # fps = self.cap_pana.get(cv2.CAP_PROP_FPS)
        # print 'v4l2src device=' + dev_pana
        # print "Frames per second using video.get(cv2.cv.CV_CAP_PROP_FPS): {0}".format(fps)

        # self.cap_webcam.append(cv2.VideoCapture(
        #     'v4l2src device=' + dev_webcam[0] + ' ! image/jpeg,width=960,height=544,framerate=30/1 ! decodebin ! videoconvert ! appsink'))
        # fps = self.cap_webcam[0].get(cv2.CAP_PROP_FPS)
        # print 'v4l2src device=' + dev_webcam[0]
        # print "Frames per second using video.get(cv2.cv.CV_CAP_PROP_FPS): {0}".format(fps)

        # self.cap_webcam.append(cv2.VideoCapture(
        #     'v4l2src device=' + dev_webcam[1] + ' ! image/jpeg,width=960,height=544,framerate=30/1 ! decodebin ! videoconvert ! appsink'))
        # fps = self.cap_webcam[1].get(cv2.CAP_PROP_FPS)
        # print 'v4l2src device=' + dev_webcam[1]
        # print "Frames per second using video.get(cv2.cv.CV_CAP_PROP_FPS): {0}".format(fps)

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

    def callback_cam_h(self, data):
        pass

    def callback_cam_r(self, data):
        pass

    def callback_cam_l(self, data):
        pass

    def run(self):
        while self.running:
            if self.curr_cam == 'head_cam':
                ret, p_frame = self.cap_pana.read()
                frame = p_frame[268:p_frame.shape[0] - 272, 0:p_frame.shape[1]]
                self.out_send.write(frame)

            elif self.curr_cam == 'right_cam':
                ret, w_frame = self.cap_webcam[0].read()
                black = np.zeros((self.height, 480, 3), np.uint8)
                frame = np.concatenate((black, w_frame[2:w_frame.shape[0] - 2, 0:w_frame.shape[1]], black), axis=1)
                self.out_send.write(frame)
            elif self.curr_cam == 'left_cam':
                ret, w_frame = self.cap_webcam[1].read()
                black = np.zeros((self.height, 480, 3), np.uint8)
                frame = np.concatenate((black, w_frame[2:w_frame.shape[0] - 2, 0:w_frame.shape[1]], black), axis=1)
                self.out_send.write(frame)

            if not ret:
                print('empty frame')
                break

            cv2.imshow('send', frame)
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q'):
                break
            elif key & 0xFF == ord('s'):
                if flag == 0:
                    flag = 1
                elif flag == 1:
                    flag = 2
                else:
                    flag = 0
        self.running = False
        self.out_send.release()
        cv2.destroyAllWindows()
        threading.Thread.exit()

if __name__ == '__main__':
    ip_addr = '130.215.206.182'

    v_thread = videoThread(0, 'videoT', ip_addr)
    v_thread.start()
