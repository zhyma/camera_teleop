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
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_from_matrix, euler_matrix, euler_from_matrix
from geometry_msgs.msg import TransformStamped, PoseStamped, Quaternion
import tf
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Int8
import time
from math import pi
import copy

import time

# parameters for initial robot pose
q_init_left = [-0.0211407, -0.0855429, -0.908688, 1.08781, -0.518622, 1.41133, 0.531204]
q_init_right = [0.0211407, -0.0855429,  0.908688, 1.08781,  0.518622, 1.41133, -0.531204]

q_tuckarm_left = [0.58897088559570313, -0.9675583808532715, -1.2034079267211915, 1.7132575355041506, 0.6776360122741699, 1.0166457660095216, -0.5065971546203614]
q_tuckarm_right = [-0.58897088559570313, -0.9675583808532715, 1.2034079267211915, 1.7132575355041506, -0.6776360122741699, 1.0166457660095216, 0.5065971546203614]

viewToWorldScaleXY = 1

rhand_rot_init = [-pi/2, 0, pi/2]
lhand_rot_init = [pi/2, 0, -pi/2]
rhand_cam_init = [-pi/4*3, 0, pi/2]
lhand_rot_init = [pi/4*3, 0, -pi/2]
scale = [1.276, 1, 1.109]

#Configuration variable: where's the state server?
system_state_addr = EbolabotSystemConfig.getdefault_ip('state_server_computer_ip',('localhost',4568))

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
        self.axis_type = 'mirror'

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

        elif c == 'a':
            if self.axis_type == 'mirror':
                self.axis_type = 'direct'
            else:
                self.axis_type = 'mirror'

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
        
# all possible camera control mode, enumration
class ctrlModeEnu:
    # h2h: head control head, showing head camera
    # r2r: right hand control right manipulator, showing right wrist camera
    # h2r: head control right manipulator, showing right wrist camera
    # l2l: left hand control left manipulator, showing left wrist camera
    # h2l: head control left manipulator, showing left wrist camera
    h2h, r2r, h2r, l2l, h2l = range(5)
    
class anchor_status():
    def __init__(self, ee_start_rot, ee_start_loc):
        self.raw_data = None
        # raw input from vive
        self.raw_rot = euler_matrix(0, 0, 0, 'rxyz')
        self.raw_loc = np.array([0, 0, 0])
        # starting position (vive)
        # [phi, theat, psi] & [x, y, z]
        self.rot0 = euler_matrix(0, 0, 0, 'rxyz')
        self.rot0_inv = euler_matrix(0, 0, 0, 'rxyz')
        self.loc0 = np.array([0, 0, 0])
        # current position, relative to the starting position (vive)
        self.rot = euler_matrix(0, 0, 0)
        self.loc = np.array([0, 0, 0])
        # last position before change the cam status
        self.rot_last = euler_matrix(0, 0, 0, 'rxyz')
        self.loc_last = np.array([0, 0, 0])
        # starting end-effector position (robot)
        # fixed value
        self.ee_rot0 = euler_matrix(ee_start_rot[0], ee_start_rot[1], ee_start_rot[2], 'rxyz')
        self.ee_loc0 = np.array(ee_start_loc)
        # current end-effector location
        self.ee_rot = self.ee_rot0.copy()
        self.ee_loc = self.ee_loc0.copy()
        self.ee_rot_last = self.ee_rot0.copy()
        self.ee_loc_last = self.ee_loc0.copy()

    def reset(self):
        # take the current position (raw) as the origin of that hand/head
        self.rot0 = self.raw_rot.copy()
        self.rot0_inv = np.linalg.inv(self.rot0)
        self.loc0 = self.raw_loc.copy()
        return
   
    def callback(self, data):
        rot = data.transform.rotation
        tran = data.transform.translation
        rot = quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
        loc = np.array([tran.x, tran.y, tran.z])
        self.raw_rot = rot.copy()
        self.raw_loc = loc.copy()
        self.rot = np.matmul(self.rot0_inv, rot)
        self.loc = self.raw_loc-self.loc0
        
        self.raw_data = copy.deepcopy(data)
        return

    def snapshot(self, rot=None):
        self.loc_last = self.loc.copy()
        self.ee_loc_last = self.ee_loc.copy()

        self.rot_last = self.rot.copy()
        if isinstance(rot, list) and len(rot)==3:
            self.ee_rot_last = self.euler_matrix(rot[0], rot[1], rot[2], 'rxyz')
        else:
            self.ee_rot_last = self.ee_rot.copy()
        return

# eye-to-hand hand and cam ctrl, calculated by using matrices
def direct_ctrl(anchor):
    # R = inv(Hand_0) * Hand_t
    # final rotation should be: R_t = R * Ree_offset
    anchor.ee_rot = np.matmul(anchor.rot, anchor.ee_rot0)
    # Translation will be done under base frame. No need to rotate it.
    anchor.ee_loc = anchor.loc + anchor.ee_loc0
    return

# eye-in-hand camera ctrl
def eih_cam_ctrl(head, hand):
    # match head orientation to camera's
    phi, theta, psi = euler_from_matrix(head.rot, 'rxyz')
    r_new = euler_matrix(-psi, theta, 0,'rxyz')
    hand.ee_rot = np.matmul(hand.ee_rot_last, r_new)

    # match head translation to camera's
    cam_rot = np.array([[0, 0, -1, 0],[0, 1, 0, 0],[1, 0, 0, 0], [0, 0, 0, 1]])
    #get the rotation of the translation
    tran_rot = np.matmul(hand.ee_rot_last, cam_rot)
    t_new = np.matmul(tran_rot, np.append(head.loc, 0).transpose())[:3]
    hand.ee_loc = hand.ee_loc_last+t_new

    return

# eye-in-hand hand ctrl
def eih_task_ctrl(cam_hand, op_hand, axis_type='mirror'):
    frame_rot = np.array([[0, 0, -1, 0],[0, 1, 0, 0],[-1, 0, 0, 0], [0, 0, 0, 1]])
    cam_rot = np.matmul(cam_hand.ee_rot0, frame_rot)
    #phi, theta, psi = euler_from_matrix(cam_rot, 'rxyz')
    # ignore phi and theta, assume that camera plane is vertical to the ground

    # IF rotation IS NOT affected by camera rotation
    # rotation still use the operation hand's frame
    if axis_type == 'direct':
        op_hand.ee_rot = np.matmul(op_hand.rot, op_hand.ee_rot0)
        op_hand.ee_loc = op_hand.loc + op_hand.ee_loc_last
    else:
        ## IF rotation IS affected by camera rotation
        phi, theta, psi = euler_from_matrix(op_hand.rot)
        phi_new = -psi
        theta_new = theta
        psi_new = -phi

        r = euler_matrix(phi_new, theta_new, psi_new, 'rxyz')
        op_hand.ee_rot = np.matmul(op_hand.ee_rot0, r)

        # operation hand translation
        t_new = np.matmul(cam_rot, np.append(op_hand.loc-op_hand.loc_last, 0).transpose())[:3]
        op_hand.ee_loc = t_new + op_hand.ee_loc_last

    return cam_rot

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

        self.hm = PoseStamped()
        pos = self.hm.pose.position
        [pos.x, pos.y, pos.z] = [0, 0, 0]

        self.hand_r = anchor_status(rhand_rot_init, [0.5647,-0.18159,1.1294])
        self.hand_l = anchor_status(lhand_rot_init, [0.5647,0.18159,1.1294])
        self.head = anchor_status([0, pi/6, 0], [0, 0, 0])

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

        rospy.Subscriber('/Left_Hand', TransformStamped, self.hand_l.callback, queue_size = 1)
        rospy.Subscriber('/Right_Hand', TransformStamped, self.hand_r.callback, queue_size = 1)
        rospy.Subscriber('/vive/twist5', TransformStamped, self.head.callback, queue_size=1)
        rospy.Subscriber('/vive/controller_LHR_FF7FBBC0/joy', Joy, self.callback_vive_r, queue_size = 1)
        rospy.Subscriber('/vive/controller_LHR_FFFB7FC3/joy', Joy, self.callback_vive_l, queue_size = 1)
        self.pub_cam = rospy.Publisher('/cam_select', String, queue_size=1)
        self.pub_state = rospy.Publisher('/vive_ctrl_status', String, queue_size=1)
        self.pub_cam_ctrl = rospy.Publisher('/cam_base_ctrl', PoseStamped, queue_size=1)
        self.pub_r = rospy.Publisher('/right/UbirosGentle', Int8, queue_size=1)
        self.pub_l = rospy.Publisher('/left/UbirosGentle', Int8, queue_size=1)
        rospy.init_node('talker', anonymous=True)
        
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

    ## MAIN LOGIC
    def do_logic(self):
        ''' retract arm to initial position '''
        self._status = self.plugin.axis_type
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
                        self.hand_l.reset()
                        self.hand_r.reset()
                        self.head.reset()

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
                        # l2l -> h2l, head ctrl left cam
                        # cam  hand: take a snapshot (head ctrl)
                        # task hand: do nothing
                        self.plugin.ctrlMode = ctrlModeEnu.h2l
                        self.hand_l.snapshot()
                        #self.hand_r.snapshot()
                        self.pub_cam.publish(String("h2l"))
                        print "head control left wrist camera"
                    else:
                        direct_ctrl(self.hand_r)
                        if self.plugin.ctrlMode == ctrlModeEnu.h2l:
                            # h2l -> l2l
                            # cam  hand: do nothing
                            # task hand: do nothing
                            pass
                            #self.hand_l.snapshot(rhand_rot_init)
                        else:
                            # h2h/r2r/h2r -> l2l
                            # cam  hand: take a snapshot
                            # task hand: take a snapshot
                            self.hand_l.snapshot(lhand_cam_init)
                            self.hand_r.snapshot(rhand_rot_init)
                        self.plugin.ctrlMode = ctrlModeEnu.l2l
                        self.pub_cam.publish(String("l2l"))

                        self.hand_r.snapshot(rhand_rot_init)
                        print "left hand control left wrist camera"

                # right grip triggered
                elif self.vive_base_button_r[3] == 1: 
                    while self.vive_base_button_r[3] == 1:
                        time.sleep(0.5)
                    if self.plugin.ctrlMode == ctrlModeEnu.r2r:
                        self.plugin.ctrlMode = ctrlModeEnu.h2r

                        self.hand_r.snapshot()
                        self.hand_l.snapshot()
                        self.pub_cam.publish(String("h2r"))
                        print "head control right wrist camera"
                    else:
                        self.plugin.ctrlMode = ctrlModeEnu.r2r
                        self.pub_cam.publish(String("r2r"))
                        direct_ctrl(self.hand_l)
                        self.hand_l.snapshot(lhand_rot_init)
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
            ## HEAD control HEAD cam
            if self.plugin.ctrlMode == ctrlModeEnu.h2h:
                # the declaration of hm is at init of this function
                direct_ctrl(self.hand_r)
                direct_ctrl(self.hand_l)
                direct_ctrl(self.head)
                ori = self.hm.pose.orientation
                [ori.x, ori.y, ori.z, ori.w] = quaternion_from_matrix(self.head.ee_rot)
                self.pub_cam_ctrl.publish(self.hm)

            ## HEAD control RIGHT HAND cam
            elif self.plugin.ctrlMode == ctrlModeEnu.h2r:
                eih_cam_ctrl(self.head, self.hand_r)
                eih_task_ctrl(self.hand_r, self.hand_l, self.plugin.axis_type)
                
            ## RIGHT HAND control RIGHT HAND cam
            elif self.plugin.ctrlMode == ctrlModeEnu.r2r:
                direct_ctrl(self.hand_r)
                eih_task_ctrl(self.hand_r, self.hand_l, self.plugin.axis_type)

            ## HEAD control LEFT HAND cam
            elif self.plugin.ctrlMode == ctrlModeEnu.h2l:
                eih_cam_ctrl(self.head, self.hand_l)
                eih_task_ctrl(self.hand_l, self.hand_r, self.plugin.axis_type)

            ## LEFT HAND control LEFT HAND cam
            elif self.plugin.ctrlMode == ctrlModeEnu.l2l:
                direct_ctrl(self.hand_l)
                eih_task_ctrl(self.hand_l, self.hand_r, self.plugin.axis_type)

            rot_l = self.hand_l.ee_rot[:3, :3]
            rot_r = self.hand_r.ee_rot[:3, :3]
            tran_l = list(self.hand_l.ee_loc)
            tran_r = list(self.hand_r.ee_loc)

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

