# Import Modules
import os, sys, rospy
ebolabot_root = os.getenv("EBOLABOT_PATH",".")
sys.path.append(ebolabot_root)
from Common.system_config import EbolabotSystemConfig
from task_generator import TaskGenerator
from UI.utils.gltexture import *

from klampt import RobotPoser
from klampt.math import so3,se3,vectorops

from OpenGL.GL import *
from klampt.vis import gldraw
from klampt.vis.glinterface import GLPluginInterface as GLPluginBase
from klampt.vis.glcommon import GLWidgetPlugin

from visualization_msgs.msg import Marker
import threading
from sspp.service import Service
from sspp.topic import TopicListener
import asyncore

import numpy as np
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_from_matrix, euler_matrix
from geometry_msgs.msg import TransformStamped, PoseStamped, Quaternion
import tf
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Int8
import time
from math import pi

import time

# parameters for initial robot pose
q_init_left = [-0.0211407, -0.0855429, -0.908688, 1.08781, -0.518622, 1.41133, 0.531204]
q_init_right = [0.0211407, -0.0855429,  0.908688, 1.08781,  0.518622, 1.41133, -0.531204]

q_tuckarm_left = [0.58897088559570313, -0.9675583808532715, -1.2034079267211915, 1.7132575355041506, 0.6776360122741699, 1.0166457660095216, -0.5065971546203614]
q_tuckarm_right = [-0.58897088559570313, -0.9675583808532715, 1.2034079267211915, 1.7132575355041506, -0.6776360122741699, 1.0166457660095216, 0.5065971546203614]


viewToWorldScaleXY = 1

#Configuration variable: where's the state server?
system_state_addr = EbolabotSystemConfig.getdefault_ip('state_server_computer_ip',('localhost',4568))

#imaging stuff

# all possible camera control mode, enumration
class ctrlModeEnu:
    # h2h: head control head, showing head camera
    # r2r: right hand control right manipulator, showing right wrist camera
    # h2r: head control right manipulator, showing right wrist camera
    # l2l: left hand control left manipulator, showing left wrist camera
    # h2l: head control left manipulator, showing left wrist camera
    h2h, r2r, h2r, l2l, h2l = range(5)

class MyWidgetPlugin(GLWidgetPlugin):
    def __init__(self,taskGen):
        GLWidgetPlugin.__init__(self)
        self.world = taskGen.world
        self.sendMilestone = False
        self.initialPose = False
        self.tuckArmPose = False
        self.selectedLimb = "both"
        self.gripperControl = False
        self.gripperState = "open"
        self.trackPosition = False
        self.trackPosition_lastState = False
        self.grasp=0.0
        self.viveControl = False
        self.ctrlMode = ctrlModeEnu.h2h

    def initialize(self):
        GLWidgetPlugin.initialize(self)
        self.images = {}
        self.images['left'] = GLTexture("UI/Resources/left-arm.png")
        self.images['right'] = GLTexture("UI/Resources/right-arm.png")
        self.images['both'] = GLTexture("UI/Resources/both-arm.png")
        self.images['vive_ctrl'] = GLTexture("UI/Resources/sync.png")
        self.images['vive_ctrl_stop'] = GLTexture("UI/Resources/Off.png")

        self.images[ctrlModeEnu.h2h] = GLTexture("UI/Resources/h2h.png")
        self.images[ctrlModeEnu.h2r] = GLTexture("UI/Resources/h2r.png")
        self.images[ctrlModeEnu.h2l] = GLTexture("UI/Resources/h2l.png")
        self.images[ctrlModeEnu.r2r] = GLTexture("UI/Resources/r2r.png")
        self.images[ctrlModeEnu.l2l] = GLTexture("UI/Resources/l2l.png")
        return True

    def display(self):

        robot = self.world.robot(0)
        oldColors = []
        for i in range(robot.numLinks()):
            #c = robot.link(i).appearance().getColor()
            c = [0.5,0.5,0.5,1.0]
            oldColors.append(c)
            robot.link(i).appearance().setColor(c[0],c[1],c[2],0.5)
        GLWidgetPlugin.display(self)
        for i in range(robot.numLinks()):
            c = oldColors[i]
            robot.link(i).appearance().setColor(c[0],c[1],c[2],c[3])

    def display_screen(self):
        glRasterPos(20,30)
        glColor3f(1,1,1)
        glDisable(GL_LIGHTING)
        if self.selectedLimb in self.images:
            self.images[self.selectedLimb].blit(20,40)
        if self.viveControl == False:
            self.images['vive_ctrl_stop'].blit(100,40)
        else:
            try:
                self.ctrlMode
            except:
                self.images['vive_ctrl'].blit(100,40)
                return
                
            self.images[self.ctrlMode].blit(100,40)
                

    def print_usage(self):
        print "keyboard functions: "
        print "s - swtich between left arm, right arm, and both arm"
        print "g - swtich between open and close gripper"
        print "i - set arm to initial position"
        print "p - switch between start and stop position tracking"
        print "f - switch between preshapes/grasp modes"
    def keyboardfunc(self,c,x,y):
        if c==' ':
            self.sendMilestone = True
        # == initialization
        elif c == 'i':
            self.initialPose = True
            print "Set to initial position: Untuck arm"

        # key "v" is used to control 
        elif c == 'v':
            self.viveControl = not self.viveControl


    def eventfunc(self,type,args):
        """TODO: connect this up to GUI buttons"""
        if type=='button':
            if args=='send':
                self.sendMilestone = True


class ServiceThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self._kill = False
        self.updateFreq = 60
        self.eeGetter_left = TopicListener(system_state_addr,'.robot.endEffectors.0.xform.sensed', self.updateFreq)
        self.eeGetter_right = TopicListener(system_state_addr,'.robot.endEffectors.1.xform.sensed', self.updateFreq)

    def run(self):
        while(not self._kill):
            asyncore.loop(timeout = 1.0/self.updateFreq, count=1)
    def kill(self):
        self._kill = True

class ViveCamCtrlTaskGenerator(TaskGenerator):
    """Allows the user to interact with the model by right clicking
    using the mouse and pressing space bar to send the milestone.

    Also demonstrates how to write a Python plugin task with an OpenGL
    widget.
    """

    def __init__(self):
        # l, r, h stands for left hand, right hand, and head
        self.plugin = None
        self.q = None

        self.x = None
        self.y = None
        self.z = None
        self.w = None
        self.rot_matrix_l = euler_matrix(0, 0, 0)[:3,:3]
        self.rot_matrix_r = euler_matrix(0, 0, 0)[:3,:3]
        self.loc_l = [0.6647,0.18159,1.3294]
        self.loc_r = [0.6647,-0.18159,1.3294]
        self.loc_h = [0, 0, 0]
        self.rot_matrix_h = euler_matrix(0, 0, 0)[:3,:3]

        self.loc_l_org = [0.6647,0.18159,1.3294]
        self.loc_r_org = [0.6647,-0.18159,1.3294]
        self.rot_l_org = euler_matrix(pi/2, pi/2, 0)[:3,:3]
        self.rot_r_org = euler_matrix(-pi/2, pi/2, 0)[:3,:3]

        self.loc_l_offset = [0, 0, 0]
        self.rot_l_offset = np.eye(3)
        self.loc_r_offset = [0, 0, 0]
        self.rot_r_offset = np.eye(3)
        self.loc_h_offset = [0, 0, 0]
        self.rot_h_offset = np.eye(3)
        self.scale = [1.276, 1, 1.109]

        self.loc_l_last = [0, 0, 0]
        self.rot_l_last = np.eye(3)
        self.loc_r_last = [0, 0, 0]
        self.rot_r_last = np.eye(3)

        self.head_motion = PoseStamped()
        self.head_motion.pose.position.x = 0
        self.head_motion.pose.position.y = 0
        self.head_motion.pose.position.z = 0
        self.head_motion.pose.orientation.x = 0
        self.head_motion.pose.orientation.y = 0
        self.head_motion.pose.orientation.z = 0
        self.head_motion.pose.orientation.w = 1

        self.vive_base_button = [0,0,0,0]
        self.vive_base_axes = [0,0,0]
        self.vive_rot_button = [0,0,0,0]
        self.grippercent_l = 0
        self.grippercent_r = 0
        self.gripperlock_l = False
        self.gripperlock_r = False
        self.vive_rot_axes = [0,0,0]

        self.vive_base_axes_r = [1.0, 1.0, 1.0]
        self.vive_base_axes_l = [1.0, 1.0, 1.0]
        self.vive_base_button_r = [0, 0, 0, 0]
        self.vive_base_button_l = [0, 0, 0, 0]
        # A timer to remove possible false trigger of buttons
        self.state_time = time.time()

        rospy.Subscriber('/Left_Hand', TransformStamped, self.callback_arm_location_l, queue_size = 1)
        rospy.Subscriber('/Right_Hand', TransformStamped, self.callback_arm_location_r, queue_size = 1)
        rospy.Subscriber('/vive/twist5', PoseStamped, self.callback_head_location, queue_size=1)
        rospy.Subscriber('/vive/controller_LHR_FF7FBBC0/joy', Joy, self.callback_vive_r, queue_size = 1)
        rospy.Subscriber('/vive/controller_LHR_FFFB7FC3/joy', Joy, self.callback_vive_l, queue_size = 1)
        self.pub_cam = rospy.Publisher('/cam_select', String, queue_size=1)
        self.pub_state = rospy.Publisher('/vive_ctrl_status', String, queue_size=1)
        self.pub_cam_ctrl = rospy.Publisher('/cam_base_ctrl', PoseStamped, queue_size=1)
        self.pub_r = rospy.Publisher('/right/UbirosGentle', Int8, queue_size=1)
        self.pub_l = rospy.Publisher('/left/UbirosGentle', Int8, queue_size=1)
        rospy.init_node('talker', anonymous=True)
        self.cnt = 0
        
        self.pub_state.publish(String('pause'))
        self.pub_cam.publish(String('h2h'))

    def callback_vive_l(self,data):
        self.vive_base_button_l = data.buttons
        self.vive_base_axes_l = data.axes
        self.grippercent_l = self.vive_base_axes_l[2]*100
    
    def callback_vive_r(self,data):
        self.vive_base_button_r = data.buttons
        self.vive_base_axes_r = data.axes
        self.grippercent_r = self.vive_base_axes_r[2]*100
    
    
    def callback_arm_location_l(self,data):
        rot = data.transform.rotation
        tran = data.transform.translation
        self.loc_l = [tran.x, tran.y, tran.z]
        self.rot_matrix_l = quaternion_matrix([rot.x, rot.y, rot.z, rot.w])[:3,:3]
    
    def callback_arm_location_r(self,data):
        rot = data.transform.rotation
        tran = data.transform.translation
        self.loc_r = [tran.x, tran.y, tran.z]
        self.rot_matrix_r = quaternion_matrix([rot.x, rot.y, rot.z, rot.w])[:3,:3]

    def callback_head_location(self,data):
        rot = data.pose.orientation
        tran = data.pose.position
        self.loc_h = [tran.x, tran.y, tran.z]
        self.rot_matrix_h = quaternion_matrix([rot.x, rot.y, rot.z, rot.w])[:3,:3]
        self.head_motion = data

    def name(self): return "Vive Hybrid Control mode (head and hand)"

    def init(self,world):
        self.world = world
        self.robotPoser = RobotPoser(world.robot(0))
        self.plugin = MyWidgetPlugin(self)
        self.plugin.addWidget(self.robotPoser)
        self.robot = world.robot(0)

        left_arm_link_names = ['left_upper_shoulder','left_lower_shoulder','left_upper_elbow','left_lower_elbow','left_upper_forearm','left_lower_forearm','left_wrist']
        right_arm_link_names = ['right_upper_shoulder','right_lower_shoulder','right_upper_elbow','right_lower_elbow','right_upper_forearm','right_lower_forearm','right_wrist']
        base_link_names = ['base_x','base_y','base_yaw']
        self.left_arm_link_indices = [self.robot.link(l).index for l in left_arm_link_names]
        self.right_arm_link_indices = [self.robot.link(l).index for l in right_arm_link_names]
        self.base_link_indices = [self.robot.link(l).index for l in base_link_names]
        if any(v < 0 for v in self.left_arm_link_indices+self.right_arm_link_indices):
            raise ValueError("Robot in world does not have Baxter's torso?")

        ## == subscribe marker position
        #self.marker = Marker()
        #self.base_height = 1.1
        #self.marker_pos = [0.0, 0.0, 0.0]
        #self.marker_pos_tf = [0.0, 0.0, 0.0]
        ##TODO: Same, check node and init  in the base class
        ##rospy.init_node('marker_listener', anonymous=True)

        #== auto tracking parameter
        self.tracking_speed_ratio = 1.0
        self.tracking_vel = [0.0, 0.0, 0.0]
        self.left_ee_t = [0.0, 0.0, 0.0]
        self.right_ee_t = [0.0, 0.0, 0.0]
        self.reactive_vel_left_ee = [0.0, 0.0, 0.0]
        self.reactive_vel_right_ee = [0.0, 0.0, 0.0]

        return True

    def start(self):
        self._status = 'ok'
        self.robotPoser.set(self.world.robot(0).getConfig())
        self.serviceThread = ServiceThread()
        self.serviceThread.start()
        #self.glove_sub=rospy.Subscriber("/Marker_glove", Marker, self.callback, None)

        return True

    def stop(self):
        #self.glove_sub.unregister()
        if self.serviceThread:
            self.serviceThread.kill()
            print "Waiting for thread join..."
            self.serviceThread.join()
            print "Done"
            self.serviceThread = None

    def getEE(self):
        R1,t1 = self.serviceThread.eeGetter_left.get()
        R2,t2 = self.serviceThread.eeGetter_right.get()
        self.left_ee_t = list(t1)
        self.right_ee_t = list(t2)
        print "left hand EE = ", self.left_ee_t
        print "right hand EE = ", self.right_ee_t

    def cal_reactive_vel(self, pos_hand, pos_target, vel_ratio):
        reactive_vel = [0.0, 0.0, 0.0]
        pos_diff = vectorops.sub(tuple(pos_target), tuple(pos_hand))
        pos_diff_norm = vectorops.norm(pos_diff)
        if pos_diff_norm >= 0.02:
            vel = vectorops.mul(pos_diff, vel_ratio)
            reactive_vel = list(vel)
        return reactive_vel


    def get(self):
        res = self.do_logic()
        return res

    def do_logic(self):
        ''' retract arm to initial position '''
        if self.plugin.initialPose:
            self.plugin.viveControl = False
            self.plugin.initialPose = False
            self.q = self.robotPoser.get()
            if self.plugin.selectedLimb == "both":
                for i in range(len(self.left_arm_link_indices)):
                    idx = self.left_arm_link_indices[i]
                    self.q[idx] = q_init_left[i]
                for i in range(len(self.right_arm_link_indices)):
                    idx = self.right_arm_link_indices[i]
                    self.q[idx] = q_init_right[i]
                print "set both arms to initial pose"
            if self.plugin.selectedLimb == "left":
                for i in range(len(self.left_arm_link_indices)):
                    idx = self.left_arm_link_indices[i]
                    self.q[idx] = q_init_left[i]
                print "set left arms to initial pose"
            if self.plugin.selectedLimb == "right":
                for i in range(len(self.right_arm_link_indices)):
                    idx = self.right_arm_link_indices[i]
                    self.q[idx] = q_init_right[i]
                print "set right arms to initial pose"
            self.robotPoser.set(self.q)

        ''' Control - Robot arm and mobile base'''

        if self.plugin.sendMilestone:
            self.plugin.sendMilestone = False
            self.q = self.robotPoser.get()
            qcmd = self.world.robot(0).getConfig()
            baseMoved = any(qcmd[i] != self.q[i] for i in self.base_link_indices)
            leftArmMoved = any(qcmd[i] != self.q[i] for i in self.left_arm_link_indices)
            rightArmMoved = any(qcmd[i] != self.q[i] for i in self.right_arm_link_indices)
            moved = []
            if baseMoved: moved.append('base')
            if leftArmMoved: moved.append('left')
            if rightArmMoved: moved.append('right')
            targets = []
            if baseMoved: targets.append([self.q[i] for i in self.base_link_indices])
            if leftArmMoved: targets.append([self.q[i] for i in self.left_arm_link_indices])
            if rightArmMoved: targets.append([self.q[i] for i in self.right_arm_link_indices])
            print "Sending milestone",self.q,"parts",",".join(moved)
            #CONSTRUCT THE TASK HERE
            msg = {}
            msg['type'] = 'JointPose'
            msg['parts'] = moved
            msg['positions'] = targets
            msg['speed'] = 1
            msg['safe'] = 0
            return msg

        # ----------------------------------------------------------------
        # This part uses vive buttons to select the state
        # Use menu key of the right controllers to start
        if self.vive_base_button_r[0] == 1:
            new_time = time.time()
            if new_time - self.state_time > 0.5:
                while self.vive_base_button_r[0] == 1:
                    time.sleep(0.5)
                # start teleoperating the robot
                if self.plugin.viveControl == False:
                    self.pub_state.publish(String('controlling'))
                    self.plugin.viveControl = True
                    self.state_time = new_time
                    ## ----RESET THE POSE---- ##
                    # if trigger is pulled, reset arm position and orientation,
                    # otherwise, 
                    if self.vive_base_button_r[1] == 1:
                        print "reset position and orientation of both controllers"
                        # get the offset of the RIGHT controller
                        self.loc_r_offset = [i for i in self.loc_r]
                        # rotation offset is R_curr^{-1}
                        self.rot_r_offset = np.linalg.inv(self.rot_matrix_r)

                        # get the offset of the LEFT controller
                        self.loc_l_offset = [i for i in self.loc_l]
                        # rotation offset is R_curr^{-1}
                        self.rot_l_offset = np.linalg.inv(self.rot_matrix_l)

                        # get the offset of the HEAD
                        self.loc_h_offset = [i for i in self.loc_h]
                        self.rot_h_offset = np.linalg.inv(self.rot_matrix_h)

                # stop teleoperating the robot
                else:
                    self.pub_state.publish(String('pause'))
                    self.plugin.viveControl = False

            return None

        # Use the left menu button and grip buttons (both side) to switch between hand control hand motion
        # and head control hand motion
        if self.vive_base_button_l[0] == 1 or self.vive_base_button_l[3] == 1 or self.vive_base_button_r[3] == 1:
            new_time = time.time()
            if new_time - self.state_time > 0.5:
                # left grip triggered
                if self.vive_base_button_l[3] == 1:
                    while self.vive_base_button_l[3] == 1:
                        time.sleep(0.5)
                    
                    if self.plugin.ctrlMode == ctrlModeEnu.l2l:
                        # step "in" to the head control wrist mode
                        self.plugin.ctrlMode = ctrlModeEnu.h2l
                        # save current end-effector state
                        rot_l = np.matmul(self.rot_l_offset, self.rot_matrix_l)
                        rot_l = np.matmul(rot_l, self.rot_l_org)
                        tran_l = [(self.loc_l[i] - self.loc_l_offset[i]) * self.scale[i] + self.loc_l_org[i] for i in range(3)]
                        self.loc_l_last = tran_l
                        self.rot_l_last = rot_l
                        self.pub_cam.publish(String("h2l"))
                        print "head control left wrist camera"
                    else:
                        self.plugin.ctrlMode = ctrlModeEnu.l2l
                        self.pub_cam.publish(String("l2l"))
                        print "left hand control left wrist camera"

                # right grip triggered
                elif self.vive_base_button_r[3] == 1: 
                    while self.vive_base_button_r[3] == 1:
                        time.sleep(0.5)
                    if self.plugin.ctrlMode == ctrlModeEnu.r2r:
                        self.plugin.ctrlMode = ctrlModeEnu.h2r
                        rot_r = np.matmul(self.rot_r_offset, self.rot_matrix_r)
                        rot_r = np.matmul(rot_r, self.rot_r_org)
                        tran_r = [(self.loc_r[i] - self.loc_r_offset[i]) * self.scale[i] + self.loc_r_org[i] for i in range(3)]
                        self.loc_r_last = tran_r
                        self.rot_r_last = rot_r
                        self.pub_cam.publish(String("h2r"))
                        print "head control right wrist camera"
                    else:
                        self.plugin.ctrlMode = ctrlModeEnu.r2r
                        self.pub_cam.publish(String("r2r"))
                        print "right hand control right wrist camera"

                # switch to head control head camera for all other cases
                else:
                    while self.vive_base_button_l[0] == 1:
                        time.sleep(0.5)
                    self.pub_cam.publish(String("h2h"))
                    self.plugin.ctrlMode = ctrlModeEnu.h2h


        # ----------------------------------------------------------------
        # This part uses vive controllers to control the end effectors
        if self.plugin.viveControl == True:
            if self.plugin.ctrlMode == ctrlModeEnu.h2h:
                rot_h = np.matmul(self.rot_h_offset, self.rot_matrix_h)
                rot_h = np.append(rot_h, [[0, 0, 0]], 0)
                rot_h = np.append(rot_h, [[0], [0], [0], [1]], 1)
                hm = PoseStamped()
                pos = hm.pose.position
                [pos.x, pos.y, pos.z] = [0, 0, 0]
                ori = hm.pose.orientation
                [ori.x, ori.y, ori.z, ori.w] = quaternion_from_matrix(rot_h)
                self.pub_cam_ctrl.publish(hm)

            # if right wrist is controlled by head motion
            if self.plugin.ctrlMode == ctrlModeEnu.h2r:
                rot_r = np.matmul(self.rot_h_offset, self.rot_matrix_h)
                #rot_r = np.matmul(rot_r, self.rot_r_last)
                rot_r = np.matmul(self.rot_r_last, rot_r)
                #tran_r = [(self.loc_h[i] - self.loc_h_offset[i]) * self.scale[i] + self.loc_r_last[i] for i in range(3)]
                tran_r = [(self.loc_h[i] - self.loc_h_offset[i]) * 1.0 + self.loc_r_last[i] for i in range(3)]
            # otherwise use left hand motion control left hand (default)
            else: 
                # Expected orientation is R_init_hand_offset^(-1) * R_current_vive_pose * R_init_robot_pose
                # Note that rot_r_offset has already been inversed.
                rot_r = np.matmul(self.rot_r_offset, self.rot_matrix_r)
                rot_r = np.matmul(rot_r, self.rot_r_org)
                tran_r = [(self.loc_r[i] - self.loc_r_offset[i]) * self.scale[i] + self.loc_r_org[i] for i in range(3)]

            # if LEFT wrist is controlled by HEAD motion
            if self.plugin.ctrlMode == ctrlModeEnu.h2l:
                rot_l = np.matmul(self.rot_h_offset, self.rot_matrix_h)
                rot_l = np.matmul(rot_l, self.rot_l_last)
                tran_l = [(self.loc_h[i] - self.loc_h_offset[i]) * 1.0 + self.loc_l_last[i] for i in range(3)]
            # otherwise use LEFT hand motion control LEFT HAND (default)
            else:
                rot_l = np.matmul(self.rot_l_offset, self.rot_matrix_l)
                rot_l = np.matmul(rot_l, self.rot_l_org)
                tran_l = [(self.loc_l[i] - self.loc_l_offset[i]) * self.scale[i] + self.loc_l_org[i] for i in range(3)]

            # base movement and grippers are not effected
            if self.vive_base_button_l[2] == 1 or self.vive_base_button_r[2] == 1:
                self.baseCommandVelocity = [viewToWorldScaleXY*float(self.vive_base_axes_l[1])/5,-viewToWorldScaleXY*float(self.vive_base_axes_l[0])/5,-float(self.vive_base_axes_r[0])/5]
            else:
                self.baseCommandVelocity = [0, 0, 0]
            pos_msg = {"type" : "CartesianPoseBase",
                "limb":"both", "position": tran_l + tran_r,
                "rotation" : np.linalg.inv(rot_l).flatten().tolist() + np.linalg.inv(rot_r).flatten().tolist(), 
                "speed":1,
                "maxJointDeviation":0.5,
                "safe":0,
                'base_velocity': self.baseCommandVelocity
            }
            
            # Gripper action: pull the trigger will close it in precentage, when button is triggered, lock.
            # when trigger the button again, release the gripper.
            if self.vive_base_button_l[1] == 1:
                while self.vive_base_button_l[1] == 1:
                    time.sleep(0.2)  
                self.gripperlock_l = not self.gripperlock_l
                self.pub_l.publish(100)
                
            if self.vive_base_button_r[1] == 1:
                while self.vive_base_button_r[1] == 1:
                    time.sleep(0.2)
                self.gripperlock_r = not self.gripperlock_r
                self.pub_r.publish(100)
                
            if not self.gripperlock_l:
                self.pub_l.publish(self.grippercent_l)
            if not self.gripperlock_r:
                self.pub_r.publish(self.grippercent_r)

            return pos_msg

        return None


    def glPlugin(self):
        return self.plugin

def make():
    return ViveCamCtrlTaskGenerator()


# === send raw_joint_position command
