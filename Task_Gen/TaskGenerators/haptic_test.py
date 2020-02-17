# Import Modules
from task_generator import TaskGenerator
import rospy
import time
import os
import signal
import sys
ebolabot_root = os.getenv("EBOLABOT_PATH",".")
sys.path.append(ebolabot_root)
from Common.system_config import EbolabotSystemConfig
import threading
import copy
import math

from numpy import clip
from itertools import cycle
from enum import Enum

from OpenGL.GL import *
from klampt.vis import gldraw
from klampt.vis.glinterface import GLPluginInterface as GLPluginBase
from klampt.vis.glcommon import GLWidgetPlugin
import subprocess

from visualization_msgs.msg import Marker

from klampt.math import so3,se3,vectorops
from sspp.service import Service
from sspp.topic import MultiTopicListener
import asyncore

from UI.utils.gltexture import *
import csv
import rosbag
from std_msgs.msg import Int32, String
from UI.utils.gripper_controller import *
# import baxter_external_devices

import baxter_interface
#
# try:
#     from PIL import Image
# except ImportError, err:
#     import Image

from baxter_interface import CHECK_VERSION

#Configuration variable: where's the state server?
system_state_addr = EbolabotSystemConfig.getdefault_ip('state_server_computer_ip',('localhost',4568))

SIDES=['left','right','both']

GRIPPER_OPEN = 1.0
GRIPPER_CLOSE = 0.2
NUM_FINGER_SETPOINTS = 4
GRIPPER_RES = GRIPPER_OPEN - GRIPPER_CLOSE / (NUM_FINGER_SETPOINTS-1)
haptic_switch = "/transcript"
#Configuration variable: where's the haptic service?


class LimbPose(Enum): # TODO: Move to GLWidget or base taskGen
     FREE = 0
     TUCK = 1
     UNTUCK = 2
     PUSH = 3

try:
    #default 'tcp://192.168.1.128:3456' ?
    haptic_service_addr = EbolabotSystemConfig.get_ip('haptic_server_computer_ip')

except Exception:
    print "Haptic device server not configured. Please configure the appropriate variable in",EbolabotSystemConfig.system_config_fn
    exit(1)

#modes = ['normal','gripper', 'tracking', 'switchPose', 'basePosition', 'baseOrientation', 'gripperIncremental', 'sync']
simple_modes= cycle(['arm', 'base', 'gripper', 'gripper_pinch','camera'])


#set this to -1 to have view-centric control, looking at the face of the robot
viewToWorldScaleXY = 1

defaultDeviceState = {'mode':'normal',
                      'time': 0.0,
                      'buttonDown':False,
                      'position': [0.0, 0.0, 0.0],
                      'rotationMoment': [0.0, 0.0, 0.0],
                      'jointAngle': [0.0, 0.0, 0.0],
                      'gimbalAngle': [0.0, 0.0, 0.0],
                      "velocity": [0.0, 0.0, 0.0],
                      "angularVelocity": [0.0, 0.0, 0.0],
                      'rotationScale':3,
                      'positionScale':7.5,
                      'deviceInitialTransform': None,
                      'devicePreviousTransform': None,
                      'deviceCurrentTransform': None,
                      'linearVel': [0.0, 0.0, 0.0],
                      'angularVel': [0.0, 0.0, 0.0],
                      'newupdate': False,
                      'lastFingerPose': 1.0
                      }

scale_m2cm = 100
endEffectorIndices = [25,45]

#Transform from device to robot/simulation (in column format)
#in tool handle frame
#+z is backward axis, rotates around handle (roll)
#+x is right, rotation ins pitch
#+y is up, rotation is yaw
worldToHandle = ([0,-1,0,   0,0,1,  -1,0,0 ],  [0,0,0])
#in the world frame, +z is up (yaw), +y is left (pitch), and +x is forward (roll)

#then, the handle transform to the user's reference (handle pointing forward in the) transform is
handleToUser = ([-1,0,0,   0,0,1,  0,1,0],[0,0,0])
#This is the output of the user frame
worldToUser = (so3.inv([0,1,0,   0,0,1,  1,0,0 ]),  [0,0,0])
userToWorld = (so3.inv([0,1,0,   -1,0,0,  0,0,1]),   [0,0,0])

debugHapticTransform = False

def se3translation(v):
    return (so3.identity(),v)

def deviceToViewTransform(R,t):
    Ruser = so3.mul(so3.inv(handleToUser[0]),R)
    R = so3.mul(userToWorld[0],so3.mul(Ruser,worldToUser[0]))
    return (R,so3.apply([0,1,0,    0,0,1,   1,0,0],t))

def deviceToWidgetTransform(R,t):
    """Given a device transform in the viewpoint-local frame, map to the device transform in world coordinates."""
    Tview = so3.rotation((0,0,1),math.pi),[0,0,0]
    return se3.mul(Tview,deviceToViewTransform(R,t))

def baseVelocityViewToWorld(twist):
    """Given dx,dy,dheta, maps this from view-centric coordinates to world coordinates"""
    return [-twist[0],-twist[1],twist[2]]

endEffectorLocalTransforms = [(so3.identity(),(0,0,0.08)),
                              (so3.identity(),(0,0,0.08))]

''' frame transformation: from kienct frame to pedestal frame  - determined by extrinsic calibration '''
q = (0.564775,0.425383,0.426796,0.563848)
R1 = so3.from_quaternion(q)
t1 = (0.227677,0.0916972,0.0974174)
kinect_frame_to_pedestal = (R1, t1)

''' default pose:
- front: move arm to the front of robot (equal to untuck arm)
- side: default - move arm to side so the view of kinect will not be blocked; if recorded arm posture exist, switch to recorded arm posture


'''
alignArmPose = {}

alignArmPose[LimbPose.FREE] = {'left':[], 'right': []}

alignArmPose[LimbPose.UNTUCK] = {'left':[-0.08897088559570313, -0.9675583808532715, -1.2034079267211915, 1.7132575355041506, 0.6776360122741699, 1.0166457660095216, -0.5065971546203614],
                                       'right': [0.08897088559570313, -0.9675583808532715, 1.2034079267211915, 1.7132575355041506, -0.6776360122741699, 1.0166457660095216, 0.5065971546203614]}

alignArmPose[LimbPose.TUCK] = {'left':[0.192255, 0.232187, -1.63754, 0.99, 0.281169, 1.41366, 1.48787],
                                     'right': [-0.192255, 0.232187, 1.63754, 0.99, -0.281169, 1.41366, -1.48787]}


alignArmPose[LimbPose.PUSH] = {'left':[-0.160088, -1.10271, -1.12767, 1.84711, 0.629936, 0.990431, 0.81607],
                                     'right': [0.160088, -1.10271, 1.12767, 1.84711, -0.629936, 0.990431, -0.851607]}


# alignArmPose['straight'] = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
#                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]


recordArmPose = {}

recordArmPose['front'] = [[-0.08897088559570313, -0.9675583808532715, -1.2034079267211915, 1.7132575355041506, 0.6776360122741699, 1.0166457660095216, -0.5065971546203614],
                           [0.08897088559570313, -0.9675583808532715, 1.2034079267211915, 1.7132575355041506, -0.6776360122741699, 1.0166457660095216, 0.5065971546203614]]

recordArmPose['record'] = [[0.58897088559570313, -0.9675583808532715, -1.2034079267211915, 1.7132575355041506, 0.6776360122741699, 1.0166457660095216, -0.5065971546203614],
                           [-0.58897088559570313, -0.9675583808532715, 1.2034079267211915, 1.7132575355041506, -0.6776360122741699, 1.0166457660095216, 0.5065971546203614]]



armPoseSign = [-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0]
endEffectorSign = [1.0, -1.0, 1.0]


# ===========================================================


''' simple_modes: arm, base, auto, fix, joint '''
HapticMode = simple_modes.next()
RecordPoseButton = False
CollisionDetectionEnabled = False

# class GLTexture:
#
#     def __init__(self, fn=None):
#         self.glid = None
#         if fn:
#             self.loadImage(fn)
#
#
#     def destroy(self):
#         glDeleteTextures([self.glid])
#
#
#     def setBytes(self, w, h, buffer, glformat=GL_RGBA):
#         self.w, self.h = w, h
#         if self.glid == None:
#             self.glid = glGenTextures(1)
#         glBindTexture(GL_TEXTURE_2D, self.glid)
#         glPixelStorei(GL_UNPACK_ALIGNMENT, 1)
#         glTexImage2D(
#             GL_TEXTURE_2D, 0, glformat, w, h, 0,
#             glformat, GL_UNSIGNED_BYTE, buffer
#         )
#
#
#     def loadImage(self, fn):
#         im = Image.open(fn)
#         try:
#             self.w, self.h, image = im.size[0], im.size[1], im.tobytes("raw", "RGBA", 0, -1)
#         except SystemError:
#             self.w, self.h, image = im.size[0], im.size[1], im.tobytes("raw", "RGBX", 0, -1)
#         if self.glid == None:
#             self.glid = glGenTextures(1)
#         glBindTexture(GL_TEXTURE_2D, self.glid)
#         glPixelStorei(GL_UNPACK_ALIGNMENT, 1)
#         glTexImage2D(
#             GL_TEXTURE_2D, 0, GL_RGBA, self.w, self.h, 0,
#             GL_RGBA, GL_UNSIGNED_BYTE, image
#         )
#         return True
#
#
#     def enable(self, smooth=True, glmode=GL_MODULATE):
#         glEnable(GL_TEXTURE_2D)
#         if smooth:
#             glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
#             glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
#         else:
#             glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
#             glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
#         glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, glmode)
#         glBindTexture(GL_TEXTURE_2D, self.glid)
#
#
#     def disable(self):
#         glDisable(GL_TEXTURE_2D)
#
#
#     def blit(self, x, y, w=None, h=None):
#         if w == None: w = self.w
#         if h == None: h = self.h
#         self.enable()
#         glDisable(GL_LIGHTING)
#         glColor4f(1, 1, 1, 1)
#         glBegin(GL_QUADS)
#         glTexCoord2f(0, 1)
#         glVertex2f(x, y)
#         glTexCoord2f(0, 0)
#         glVertex2f(x, y + h)
#         glTexCoord2f(1, 0)
#         glVertex2f(x + w, y + h)
#         glTexCoord2f(1, 1)
#         glVertex2f(x + w, y)
#         glEnd()
#         self.disable()


class MyWidgetPlugin(GLPluginBase):
    def __init__(self,taskGen):
        GLPluginBase.__init__(self)
        self.taskGen = taskGen
        self.displayHelpFlag = False
        self.BaseControlMode = False
        self.printMessageFlag = True
        self.limb_pose={'left': LimbPose.FREE, 'right': LimbPose.FREE}

        # open log file
        timestr = time.strftime("%Y%m%d-%H%M%S")
        self.csvfile = open('data/keyboard_log'+timestr+'.csv', 'wb')
        fieldnames = ['button', 'time']
        self.haptic_csv = csv.DictWriter(self.csvfile, fieldnames=fieldnames)
        self.haptic_csv.writeheader()

    def initialize(self):
        GLPluginBase.initialize(self)
        global HapticMode
        global RecordPoseButton

        self.images = {}
        self.images['arm'] = GLTexture("UI/Resources/EndEffectorControl.png")
        self.images['gripper'] = GLTexture("UI/Resources/GripperControl.png")
        self.images['gripper_pinch'] = GLTexture("UI/Resources/GripperIncremental.png")
        self.images['base'] = GLTexture("UI/Resources/BasePosition.png")
        self.images['camera'] = GLTexture("UI/Resources/RecordPose.png")
        
        # Logging images
        self.images['log']=[None,None]

        self.images['log'][1] = GLTexture("UI/Resources/log-on.png")
        self.images['log'][0] = GLTexture("UI/Resources/log-off.png") 

        
        return True

    def keyboardfunc(self,c,x,y):

        global HapticMode
        global RecordPoseButton
        global debugHapticTransform
        c=c.lower()

        if c == 'h':
                    print (" VICON Plugin")
                    print (" T/Y: Tuck left/right arm")
                    print (" U/I: Untuck left/right arm")
                    print (" A,S,D,F: Label data")
                    print (" L: Enable/Disable Logging")
                    print (" H: Prints this message")
        elif c=='l':
            self.taskGen.log ^=1 
            print("Logging: " + str(self.taskGen.log ) )

       # if self.log:
       # elif c in ['a','s','d','f']:
            if self.taskGen.flag is c:
                self.taskGen.flag = 0
                print("No label ")
            else:
                self.taskGen.flag = c
                print("Label: " + c)
        elif c == '0':
            self.taskGen.flag = 0
        # global RecordPoseButton
        # global CollisionDetectionEnabled

        # Tuck/Untuck Arms

        elif c == 'u':
            self.limb_pose['left'] = LimbPose.UNTUCK
            if self.printMessageFlag:
                print("Untucking left arm")
                # self.printMessageFlag = False

        elif c == 'i':
            self.limb_pose['right'] =  LimbPose.UNTUCK
            if self.printMessageFlag:
                print("Untucking right arm")
                # self.printMessageFlag = False

        elif c == 't':
            self.limb_pose['left'] =  LimbPose.TUCK
            if self.printMessageFlag:
                print("Tucking left arm")
                # self.printMessageFlag = False

        elif c == 'y':
            self.limb_pose['right'] =  LimbPose.TUCK
            if self.printMessageFlag:
                print("Tucking right arm")

                # self.printMessageFlag = False
        elif c == 'j':
            self.limb_pose['left'] = LimbPose.FREE
            print("Left arm is free")
            self.printMessageFlag = True

        elif c == 'k':
            self.limb_pose['right'] = LimbPose.FREE
            print("Right arm is free")
            self.printMessageFlag = True

        elif c =='b':
            HapticMode = simple_modes.next()

        elif c == '.':
            debugHapticTransform ^= simple_modes.next()
            print("Beware! Haptic transfrom debug is toggled " + "On" if debugHapticTransform else "Off")


        else :
            self.printMessageFlag = True

        if c is not -1 or c is not '.' :
            self.haptic_csv.writerow({'button': c, 'time':time.time()})

        # elif c=='r':
        #     if RecordPoseButton == False:
        #         RecordPoseButton = True
        #     else:
        #         RecordPoseButton = False
        #     self.keyboard_csv.writerow({'button':'r','time':time.time()})

        # elif c=='c':
        #     CollisionDetectionEnabled = not CollisionDetectionEnabled
        #     self.keyboard_csv.writerow({'button':'c','time':time.time()})
        #     print "Collision detection toggled to:",CollisionDetectionEnabled


    def display(self):

        T1 = self.taskGen.getDesiredCartesianPose('left',0)
        T2 = self.taskGen.getDesiredCartesianPose('right',1)
        baseTransform = self.taskGen.world.robot(0).link(2).getTransform()
        glEnable(GL_LIGHTING)
        if T1 is not None:
            gldraw.xform_widget(se3.mul(baseTransform,T1),0.2,0.03,fancy=True,lighting=True)
        if T2 is not None:
            gldraw.xform_widget(se3.mul(baseTransform,T2),0.2,0.03,fancy=True,lighting=True)

        dstate = self.taskGen.serviceThread.hapticupdater.deviceState

        if debugHapticTransform:
            for deviceState in dstate:
                # debugging
                # this is the mapping from the handle to the user frame
                Traw = (so3.from_moment(deviceState['rotationMoment']),deviceState['position'])
                Tuser = se3.mul(se3.inv(handleToUser),Traw)
                T = se3.mul(userToWorld,se3.mul(Tuser,worldToUser))
                T = (T[0],so3.apply([0,-1,0,    0,0,1,   -1,0,0],T[1]))
                T = deviceToViewTransform(Traw[0],Traw[1])
                gldraw.xform_widget(se3.mul(se3translation([-1,0,0]),Traw),0.5,0.05,lighting=True,fancy=True)
                gldraw.xform_widget(se3.mul(se3translation([-0.5,0,0]),Tuser),0.5,0.05,lighting=True,fancy=True)
                # gldraw.xform_widget(se3.mul(se3translation([-0.5,0,0]),se3.mul(Traw,worldToHandle)),0.5,0.05,lighting=True,fancy=True)
                gldraw.xform_widget(T,0.5,0.05,lighting=True,fancy=True)
                break

    def display_screen(self):
        global HapticMode
        glRasterPos(20, 30)
        glColor3f(1, 1, 1)
        glDisable(GL_LIGHTING)
        #print(HapticMode)
        self.images[HapticMode].blit(20, 40)
        if 'log' in self.images:
            self.images['log'][self.taskGen.log].blit(20,74+40)

    def eventfunc(self,type,args):
        """TODO: connect this up to GUI buttons"""
        pass
# modes = ['normal','gripper', 'tracking', 'switchPose', 'basePosition', 'baseOrientation', 'gripperIncremental']


def toggleMode():
    global HapticMode
    HapticMode = simple_modes.next()


class HapticWidgetUpdateService (Service):
    """Reads from a HapticService to update the device state"""
    def __init__(self,addr):
        Service.__init__(self)
        self.open(addr,asServer=False)
        self.viewer = None
        self.widgetGetters = None
        self.deviceState = []
        self.numMessages = 0
        self.lastUpdateTime = time.time()
        self.sentHapticFeedback = False

    def close(self):
        if self.sentHapticFeedback:
            #turn off haptic feedback
            msg = {}
            msg['type'] = 'HapticForceCommand'
            msg['device'] = 0
            msg['enabled'] = 0
            self.sendMessage(msg)
            msg['device'] = 1
            self.sendMessage(msg)
            #loop until all messages are sent
            while self.writable():
                asyncore.loop(timeout = 0.02, count=100, map=self.map)
        Service.close(self)

    def onMessage(self,msg):
        global defaultDeviceState
        #print "Getting haptic message"
        #print msg
        self.numMessages += 1
        if msg['type'] != 'MultiHapticState':
            print "Strange message type",msg['type']
            return
        if len(self.deviceState)==0:
            print "Adding",len(msg['devices'])-len(self.deviceState),"haptic devices"
            while len(self.deviceState) < len(msg['devices']):
                self.deviceState.append(defaultDeviceState.copy())

        if len(msg['devices']) != len(self.deviceState):
            print "Incorrect number of devices in message:",len(msg['devices'])
            return

        # change overall state depending on button 2
        if 'events' in msg:
            for e in msg['events']:
                # print e['type']
                if e['type']=='b2_down':
                    toggleMode()
                    dstate = self.deviceState[0]
                    dstate['mode'] = HapticMode
                    dstate = self.deviceState[1]
                    dstate['mode'] = HapticMode
                    # print e['device'], HapticModeButton[e['device']]
                    print 'Changing devices','to state',dstate['mode']

        for i in range(len(self.deviceState)):
            dmsg = msg['devices'][i]
            dstate = self.deviceState[i]
            #  ===== read from msg =====
            # print dmsg
            dstate['position'] = dmsg['position']
            dstate['rotationMoment'] = dmsg['rotationMoment']
            dstate['velocity'] = dmsg['velocity']
            dstate['angularVelocity'] = dmsg['angularVelocity']
            dstate['jointAngles'] = dmsg['jointAngles']
            #print(dmsg)
            #dstate['gimbalAngle'] = dmsg['gimbalAngle']
            oldTime = dstate['time']
            dstate['time'] = dmsg['time']
            #print "position",dmsg['position']
            #print "rotation moment",dmsg['rotationMoment']
            #print "angular velocity",dmsg['angularVelocity']
            dstate['deviceCurrentTransform'] = deviceToWidgetTransform(so3.from_moment(dmsg['rotationMoment']),dmsg['position'])
            if dmsg['button1'] == 1:
                #drag widget if button 1 is down
                oldButtonDown = dstate['buttonDown']
                dstate['buttonDown'] = True
                #if oldButtonDown == False:
                #    print "start haptic motion ... "
                #movingmb

                if dstate['buttonDown']:
                    # --- take initial position when button 1 is pressed----
                    if not oldButtonDown:
                        dstate['devicePreviousTransform'] = dstate['deviceCurrentTransform']
                        dstate['deviceInitialTransform'] = dstate['devicePreviousTransform']
                        continue

                    newTime = dstate['time']
                    if newTime != oldTime:
                        # print "previous position = ", dstate['devicePreviousTransform'][1]
                        # print "current position = ", dstate['deviceCurrentTransform'][1]
                        timeInterval = newTime - oldTime
                        #print "========================"
                        #print "time = ", timeInterval

                        delta_Pos = vectorops.mul(vectorops.sub(dstate['deviceCurrentTransform'][1], dstate['devicePreviousTransform'][1]), dstate['positionScale'])
                        vel   = vectorops.div(delta_Pos, timeInterval)

                        delta_Moment = vectorops.mul(tuple(so3.moment(so3.mul(dstate['deviceCurrentTransform'][0], so3.inv(dstate['devicePreviousTransform'][0])))), dstate['rotationScale'])
                        angvel = vectorops.div(delta_Moment, timeInterval)

                        # print "vel = [%2.4f %2.4f %2.4f]" % (vel[0], vel[1], vel[2])
                        # print "angvel = [%2.4f %2.4f %2.4f]" % (angvel[0], angvel[1], angvel[2])
                        dstate['linearVel'] = list(vel)
                        dstate['angularVel'] = list(angvel)

                        dstate['newupdate'] = True

                #special modes set "newupdate" is true even when button is not down
                if dstate['mode'] == 'tracking' or dstate['mode'] == 'switchPose':
                    dstate['time'] = dmsg['time']
                    newTime = dstate['time']
                    if newTime != oldTime:
                        dstate['newupdate'] = True
                    else:
                        dstate['newupdate'] = False
                else:
                    pass

            else:
                dstate['buttonDown'] = False

            #end of loop, store previous transform
            dstate['devicePreviousTransform'] = dstate['deviceCurrentTransform']

    def onUpdate(self):
        t = time.time()
        #print self.numMessages,"haptic messages read over time",t-self.lastUpdateTime
        self.numMessages = 0
        self.lastUpdateTime = t

    def activateDrag(self,kD,device='all'):
        self.sentHapticFeedback = True
        if device == 'all':
            devices = [0,1]
        else:
            devices = [device]
        msg = {}
        msg['type'] = 'HapticForceCommand'
        msg['enabled'] = 1
        msg['forceDamping'] = [-kD]
        msg['forceCenter'] = [0,0,0]
        for device in devices:
            msg['device'] = device
            # print(msg)
            self.sendMessage(msg)

    def activateSpring(self,kP,kD,center=None,device='all'):
        self.sentHapticFeedback = True
        if device == 'all':
            devices = [0,1]
        else:
            devices = [device]
        msg = {}
        msg['type'] = 'HapticForceCommand'
        msg['enabled'] = 0
        msg['forceLinear'] = [-kP]
        msg['forceDamping'] = [-kD]
        if center is not None:
            msg['center'] = center
        for device in devices:
            msg['device'] = device
            if center is None:
                countdown = 10
                while self.numMessages == 0 and countdown > 0:
                    #need to wait for an update
                    time.sleep(0.05)
                    countdown -= 1
                if self.numMessages != 0:
                    #print "Setting center to",self.deviceState[device]['position']
                    msg['center'] = self.deviceState[device]['position']

            # print(msg)
            self.sendMessage(msg)


class ServiceThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self._kill = False
        self.stateUpdateFreq = 50
        #rospy.init_node("haptic_node")
        self.hapticupdater = HapticWidgetUpdateService(haptic_service_addr)

        self.state_listener = MultiTopicListener(system_state_addr,topics=None,rate=self.stateUpdateFreq)
        self.baseVelocitySensed = self.state_listener.listen('.robot.sensed.base.velocity')
        self.baseVelocityCommand = self.state_listener.listen('.robot.command.base.velocity')
        self.ArmPosition_left = self.state_listener.listen('.robot.sensed.left.q')
        self.ArmPosition_right = self.state_listener.listen('.robot.sensed.right.q')
        self.eeGetter_left = self.state_listener.listen('.robot.endEffectors.0.xform.destination')
        self.eeGetter_right = self.state_listener.listen('.robot.endEffectors.1.xform.destination')
        self.gripperGetter_left = self.state_listener.listen('.robot.gripper.left.positionCommand')
        self.gripperGetter_right = self.state_listener.listen('.robot.gripper.right.positionCommand')
        self.state_listener.setName("HapticGUIListener")

    def run(self):
        """Note: don't call ServiceThread.run(), call ServiceThread.start()"""
        #self.hapticupdater.run(1)
        while not self.hapticupdater.kill:
            #read from the haptic service
            asyncore.loop(timeout = 0.02, count=100, map=self.hapticupdater.map)
            self.hapticupdater.onUpdate()
            #listen to state topics
            asyncore.loop(timeout = 1.0/self.stateUpdateFreq, count=10)

        print "Closing haptic widget update service..."
        self.hapticupdater.close()
        self.hapticupdater = None

    def kill(self):
        self.hapticupdater.terminate()
        self._kill = True

    # def run(self):
    #     while(not self._kill):
    #         asyncore.loop(timeout = 1.0/self.updateFreq, count=1)
    # def kill(self):
    #     self._kill = True


class HapticTaskGenerator(TaskGenerator):
    def __init__(self):
        self.log = False

        self.serviceThread = None
        self.lastMsg = None
        self.startTransforms = [None,None]
        self.CartesianPoseControl = False
        self.last_buttonDown = [False, False]

        #=== gripper open and close control
        self.gripperStatus = [None, None]
        self.gripperControlFlag = False
        self.gripperControlTimer = [time.time(), time.time()]

        #=== gripper incremental control
        self.gripperSizeFlag = False
        self.gripperSizeTimer = [time.time(), time.time()]
        self.gripperControlRatio = 0.3
        self.gripperPosition = [[1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0]]


        self.poseStatus = [None, None]
        self.poseControlFlag = False
        self.poseControlTimer = [time.time(), time.time()]

        #  === plugin function ===
        self.plugin = None
        self.dstateMode = []

        # === base control ===
        self.baseDevice = False
        self.basePosControlFlag = False
        self.baseOrtControlFlag = False
        self.basePosVelocityRatio = 0.5
        self.baseOrtVelocityRatio = 0.25
        self.basePosCommandTimer = [time.time(), time.time()]
        self.baseOrtCommandTimer = [time.time(), time.time()]

        self.baseSensedVelocity = [0.0, 0.0, 0.0]
        self.baseCommandVelocity = [0.0, 0.0, 0.0]

        # === joint control ===
        self.jointControlRatio = 0.25

        # == arm position ===
        self.ArmPosition = [[0.0]*7, [0.0]*7]

        # == subscribe marker position
        self.marker = Marker()
        self.base_height = 1.1
        self.marker_pos = [0.0, 0.0, 0.0]
        self.marker_pos_tf = [0.0, 0.0, 0.0]
        # rospy.init_node('marker_listener', anonymous=True)

        #== auto tracking parameter
        self.autoTrackingControlFlag = False
        self.autoTrackingStatus = [None, None]
        self.autoTrackingTimer = [time.time(), time.time()]

        #== synchroniztion arm pose parameter
        self.syncControlFlag = False
        self.syncControlTimer = [time.time(), time.time()]

        #== coupling two hands parameter
        self.handCouplingControlFlag = False
        self.handCouplingControlTimer = [time.time(), time.time()]

        #== coupling two arms parameter
        self.armCouplingControlFlag = False
        self.armCouplingControlTimer = [time.time(), time.time()]

        #== arm shift parameter
        self.armShiftControlFlag = False
        self.armShiftControlTimer = [time.time(), time.time()]

        self.alignControlFlag = False
        self.alignControlTimer = [time.time(), time.time()]

        #== record pose parameter
        self.recordPoseControlFlag = False
        self.recordPoseStatus = [None, None]
        self.recordPoseTimer = [time.time(), time.time()]

        #== upper and lower limb joint control
        self.upperLimbControlFlag = False
        self.upperLimbTimer = [time.time(), time.time()]

        self.lowerLimbControlFlag = False
        self.lowerLimbTimer = [time.time(), time.time()]

        self.wristControlFlag = False
        self.wristTimer = [time.time(), time.time()]

        # flag to mark the data
        self.flag  = 0

        self.tracking_speed_ratio = 1.0
        self.robotEndEffectorPosition = [[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]]
        self.robotEndEffectorTransform = [se3.identity(),se3.identity()]
        self.EEReactiveVelocity = [[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]]
        # open log file
        timestr = time.strftime("%Y%m%d-%H%M%S")
        self.csvfile_right = open('data/haptic_right_log' + timestr + '.csv', 'wb')
        self.csvfile_left = open('data/haptic_left_log' + timestr + '.csv', 'wb')
        fieldnames = ['rotationScale', 'angularVel', 'deviceCurrentTransform', 'buttonDown', 'deviceInitialTransform', 'linearVel', 'newupdate',
                      'jointAngle', 'devicePreviousTransform', 'rotationMoment', 'position', 'gimbalAngle', 'jointAngles', 'angularVelocity',
                      'positionScale', 'mode', 'time', 'velocity', 'countVelocity', 'sumVelocity', 'lastFingerPose']
        self.haptic_right_csv = csv.DictWriter(self.csvfile_right, fieldnames=fieldnames)
        self.haptic_left_csv = csv.DictWriter(self.csvfile_left, fieldnames=fieldnames)
        self.haptic_right_csv.writeheader()
        self.haptic_left_csv.writeheader()

        self.csvfile_robot = open('data/robot_state_log_haptic' + timestr + '.csv', 'wb')
        fieldnames_robot = ['jointAngles', 'gripperStatus', 'eePosition', 'eeTransformation', 'baseSensedVelocity', 'baseCommandVelocity', 'time', 'Flag']
        self.robot_state_csv = csv.DictWriter(self.csvfile_robot, fieldnames=fieldnames_robot)
        self.robot_state_csv.writeheader()
        # gripper control
        self.gripperController = None
        self.last_position = [0.0, 0.0]

    def name(self): return "Haptic Test"

    def init(self,world):
        self.world = world
        self.robot = world.robot(0)

        self.gripperControlFlag = False
        self.gripperSizeFlag = False
        self.poseControlFlag = False
        self.syncControlFlag = False
        self.handCouplingControlFlag = False
        self.armCouplingControlFlag = False
        self.armShiftControlFlag = False
        self.alignControlFlag = False
        self.recordPoseControlFlag = False
        self.autoTrackingControlFlag = False
        self.basePosControlFlag = False
        self.baseOrtControlFlag = False
        self.upperLimbControlFlag = False
        self.lowerLimbControlFlag = False
        self.wristControlFlag = False
        return True

    def start(self):
        global haptic_switch
        self.plugin = MyWidgetPlugin(self)
        self.lastMsg = None
        if self.serviceThread==None:
            self.serviceThread = ServiceThread()
            self.serviceThread.start()
        if self.gripperController==None:
            self.gripperController = GripperController()
            self.gripperController.start()
            print "gripper controller started"
        attached = False
        for i in range(5):
            time.sleep(0.1)
            if len(self.serviceThread.hapticupdater.deviceState)!=0:
                attached = True
                break
            if i % 5 == 1:
                print "HapticTaskGenerator: Waiting for haptic device tasks to be read..."
        if not attached:
            print "Haptic server doesn't appear to be running"
            self.serviceThread.kill()
            print "Waiting for thread join..."
            self.serviceThread.join()
            print "Done"
            self.serviceThread = None
        else:
            print "Haptic server is running"
            print "Turning on drag..."
            #2 is probably the highest damping that doesnt cause vibration
            #self.serviceThread.hapticupdater.activateDrag(2)
            self.serviceThread.hapticupdater.activateSpring(kP=20,kD=0,center=None)

        self.ros_subscribers = []
        self.ros_subscribers.append(rospy.Subscriber("/Marker_glove", Marker, self.marker_glove_callback, None))
        self.switch_pub = rospy.Publisher(haptic_switch, String, queue_size=1)

        return attached

    def status(self):
        if not self.serviceThread or not self.serviceThread.hapticupdater.deviceState:
            return 'error'
        else:
            return 'ok'

    def messages(self):
        return []

    # def terminate_process_and_children(self, p):
    #     ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=subprocess.PIPE)
    #     ps_output = ps_command.stdout.read()
    #     retcode = ps_command.wait()
    #     assert retcode == 0, "ps command returned %d" % retcode
    #     for pid_str in ps_output.split("\n")[:-1]:
    #         os.kill(int(pid_str), signal.SIGINT)
    #     p.terminate()

    def stop(self):
        # self.terminate_process_and_children(self.rosbag_process)
        # self.rosbag_process.send_signal(subprocess.signal.SIGINT)
        # os.killpg(os.getpgid(self.rosbag_process.pid), signal.SIGTERM)
        if self.serviceThread:
            self.serviceThread.kill()
            print "Waiting for thread join..."
            self.serviceThread.join()
            print "Done"
            self.serviceThread = None
        if self.gripperController:
            self.gripperController.kill()
            print "gripper control thread killed"
            self.gripperController = None
        for s in self.ros_subscribers:
            s.unregister()
        self.plugin = None
        self.gripperControlFlag = False
        self.gripperSizeFlag = False
        self.poseControlFlag = False
        self.handCouplingControlFlag = False
        self.armCouplingControlFlag = False
        self.armShiftControlFlag = False
        self.alignControlFlag = False
        self.recordPoseControlFlag = False
        self.syncControlFlag = False
        self.autoTrackingControlFlag = False
        self.basePosControlFlag = False
        self.baseOrtControlFlag = False
        self.upperLimbControlFlag = False
        self.lowerLimbControlFlag = False
        self.wristControlFlag = False

    def controlMode(self):
        deviceState = self.serviceThread.hapticupdater.deviceState
        if len(deviceState)==0:
            print "No device state"
            return None

        self.dstateMode = []

        for i, dstate in enumerate(deviceState):
            self.dstateMode = self.dstateMode + [dstate['mode']]
        return self.dstateMode

    def marker_glove_callback(self, data):
        self.marker = data
        self.marker_pos[0] = self.marker.pose.position.x
        self.marker_pos[1] = self.marker.pose.position.y
        self.marker_pos[2] = self.marker.pose.position.z
        #=== transform to world coordinate
        marker = se3.apply(kinect_frame_to_pedestal, tuple(self.marker_pos))
        self.marker_pos_tf = list(marker)
        self.marker_pos_tf[2] = self.marker_pos_tf[2] + self.base_height
        # print "marker_pos_tf = ", self.marker_pos_tf

    def calReactiveVel(self, pos_hand, pos_target, vel_ratio):
        reactive_vel = [0.0, 0.0, 0.0]
        pos_diff = vectorops.sub(tuple(pos_target), tuple(pos_hand))
        pos_diff_norm = vectorops.norm(pos_diff)
        if pos_diff_norm >= 0.02:
            vel = vectorops.mul(pos_diff, vel_ratio)
            reactive_vel = list(vel)
        return reactive_vel

    def get(self):

        # === get robot status from system state service: robot end-effector position, gripper position
        self.getRobotStatus()

        # self.baseSensedVelocity = self.serviceThread.baseVelocitySensed.get()
        # self.baseCommandVelocity = self.serviceThread.baseVelocityCommand.get()

        # self.ArmPosition[0] = self.serviceThread.ArmPosition_left.get()
        # self.ArmPosition[1] = self.serviceThread.ArmPosition_right.get()

        # self.robotEndEffectorPosition[0] = t1
        # self.robotEndEffectorPosition[1] = t2

        # self.robotEndEffectorTransform[0] = T2
        # self.robotEndEffectorTransform[1] = T2

        # self.gripperPosition[0] = gripperStatus_left
        # self.gripperPosition[1] = gripperStatus_right

        if self.log:
            self.robot_state_csv.writerow({'jointAngles': self.ArmPosition, 'gripperStatus': self.gripperPosition,
                                       'eePosition': self.robotEndEffectorPosition, 'eeTransformation': self.robotEndEffectorTransform,
                                       'baseSensedVelocity': self.baseSensedVelocity, 'baseCommandVelocity': self.baseCommandVelocity,
                                       'time':int(round(time.time() * 1000)), 'Flag': self.flag})
        msg = self.do_logic()
        self.last_message = msg
        # print self.serviceThread.hapticupdater.deviceState
        # save data to file.
        if self.log:        
            self.haptic_right_csv.writerow(self.serviceThread.hapticupdater.deviceState[1])
            self.haptic_left_csv.writerow(self.serviceThread.hapticupdater.deviceState[0])
        # save robot state to file.
        # self.robot_state_bag.write(robot_state_topic, )
        # bag.write('chatter', str)

        return msg

    @staticmethod
    def isNearFingerCheckpoint(p):
        val=(p-GRIPPER_CLOSE)*(1/GRIPPER_RES)
        return abs(val-round(val))<0.03

    # def gripperPowerGrasp(self):
    #     deviceState = self.serviceThread.hapticupdater.deviceState
    #     if not len(deviceState):
    #         print "No device state"
    #         return None
    #     msg = {}
    #     p = []
    #     self.gripperControlFlag = False
    #     for i in range(0,2):
    #         if self.gripperPosition[i]:
    #             p = p + self.gripperPosition[i]
    #         else:
    #             p = p + [1.0, 1.0, 1.0, 0.4]
    #     # print self.gripperPosition
    #     for i, dstate in enumerate(deviceState):
    #         if dstate['mode'] != 'gripper': continue
    #         if dstate['buttonDown']:
    #             time_now = time.time()
    #             if time_now - self.gripperControlTimer[i] > 0.05:
    #                 if i == 1:
    #                     gripperVel = -dstate['linearVel'][1]
    #                 else:
    #                     gripperVel = dstate['linearVel'][1]

    #                 if abs(gripperVel) < 0.005:
    #                         gripperVel = 0.0

    #                 self.gripperPosition[i] =  [v + gripperVel*self.gripperControlRatio for v in self.gripperPosition[i][0:3]] + [0.3]

    #                 # print [v + gripperVel*self.gripperControlRatio for v in self.gripperPosition[i][0:3]]

    #                 for j in range(0,3):
    #                     if self.gripperPosition[i][j] > GRIPPER_OPEN:
    #                         self.gripperPosition[i][j] = GRIPPER_OPEN
    #                     elif self.gripperPosition[i][j] < GRIPPER_CLOSE:
    #                         self.gripperPosition[i][j] = GRIPPER_CLOSE

    #                 p[0+i*4:4+i*4] = self.gripperPosition[i]

    #                 self.gripperControlTimer[i] = time_now
    #                 self.gripperControlFlag = True
    #     if self.gripperControlFlag:
    #         msg['limb'] = "both"
    #         msg['position'] = p
    #         msg['type'] = 'Gripper'
    #         msg['force'] = 0.4
    #         msg['speed'] = 0.3
    #         return msg
    #     else:
    #         return None

    def gripperPinchGrasp(self, preshape):
        deviceState = self.serviceThread.hapticupdater.deviceState
        if len(deviceState)==0:
            print "No device state"
            return None
        msg = {}
        p = []
        self.gripperSizeFlag = False

        # this ensures when one of the haptic devices is not present
        # a decoy position value is added to the respective array
        # for i in range(0,2):
        #     if self.gripperPosition[i]:
        #         p = p + self.gripperPosition[i]
        #     else:
        #         p = p + [1.0, 1.0, 1.0, 1.0]
        for i, dstate in enumerate(deviceState):
            #if the dhaptic device is set to another mode, move on to the next iteration
            if dstate['mode'] not in ['gripperIncremental', 'gripper', 'gripper_pinch']: continue

            if dstate['buttonDown']:
                if not self.last_buttonDown[i]:
                    self.last_position[i] = dstate['position'][1]
                    print "set last position for device, ", i
                    print("Here")
                if dstate['newupdate']:
                    time_now = time.time()
                    if time_now - self.gripperSizeTimer[i] > 0.02:
                        gripperVel = (dstate['position'][1] - self.last_position[i]) * 5.0
                        # print gripperVel
                        self.gripperPosition[i][0:3] = [v + gripperVel for v in self.gripperPosition[i][0:3]]
                        # print self.gripperPosition[i]
                        for j in range(0, 3):
                            if self.gripperPosition[i][j] > GRIPPER_OPEN:
                                self.gripperPosition[i][j] = GRIPPER_OPEN
                            elif self.gripperPosition[i][j] < GRIPPER_CLOSE:
                                self.gripperPosition[i][j] = GRIPPER_CLOSE

                        self.gripperSizeTimer[i] = time_now
                        self.gripperSizeFlag = True
                        self.baseOrtControlFlag = False
            self.last_buttonDown[i] = dstate['buttonDown']

        if self.gripperSizeFlag:
            # print p
            # print "here"
            self.gripperPosition[0][3] = preshape
            self.gripperPosition[1][3] = preshape
            self.gripperController.send_pos(self.gripperPosition[0], 'left')
            print self.gripperPosition
            self.gripperController.send_pos(self.gripperPosition[1], 'right')
        return
        # if self.gripperSizeFlag:
        #     msg['limb'] = "both"
        #     msg['position'] = p
        #     msg['type'] = 'Gripper'
        #     msg['force'] = 0.4
        #     msg['speed'] = 0.3
        #     return msg
        # else:
        #     return None

    def getDesiredCartesianPose(self,limb,device):
        """Returns a pair (R,t) of the desired EE pose if the limb should have
        a cartesian pose message, or None if it should not.

        Implementation-wise, this reads from self.startTransforms and the deviceState
        dictionary to determine the correct desired end effector transform.  The delta
        from devices[device]['deviceCurrentTransform'] to devices[device]['deviceInitialTransform']
        is scaled, then offset by self.startTransforms[device].  (self.startTransforms is
        the end effector transform when deviceInitialTransform is set)
        """
        if limb=='left':
            T = self.serviceThread.eeGetter_left.get()
        else:
            T = self.serviceThread.eeGetter_right.get()
        if T is None:
            T = se3.identity()
        R,t=T
        deviceState = self.serviceThread.hapticupdater.deviceState
        if deviceState == None: return T
        if self.startTransforms[device] == None: return T
        dstate = deviceState[device]
        Tcur = dstate['deviceCurrentTransform']
        T0 = dstate['deviceInitialTransform']
        if T0 == None:
            T0 = Tcur
        #print "Button is down and mode is",dstate['mode']
        #print dstate
        assert T0 != None,"T0 is null"
        assert Tcur != None,"Tcur is null"
        if dstate['mode'] in ['arm', 'normal','armCoupling','armShift', 'onlyPosition']:
            relRot = so3.mul(Tcur[0],so3.inv(T0[0]))
            relTrans = vectorops.sub(Tcur[1],T0[1])
            #print "Rotation moment",so3.moment(relRot)
            translationScaling = 3.0
            if dstate['mode'] == 'onlyPosition':
                relRot = so3.from_moment([0.0, 0.0, 0.0])
            desRot = so3.mul(relRot,self.startTransforms[device][0])
            desPos = vectorops.add(vectorops.mul(relTrans,translationScaling),self.startTransforms[device][1])
            #TEST: just use start rotation
            #desRot = self.startTransforms[i][0]
            return (desRot,desPos)

        else:
            # print "how to render mode",dstate['mode'],"?"
            return T

    def BasePositionControl(self):
        deviceState = self.serviceThread.hapticupdater.deviceState
        if len(deviceState)==0:
            print "No device state"
            return None
        msg = {}
        baseVel = [0.0, 0.0, 0.0]
        self.basePosControlFlag = False
        self.baseDevice = False

        for i, dstate in enumerate(deviceState):
            if dstate['mode'] not in ['basePosition', 'base']: continue
            if self.baseDevice == False:
                if dstate['buttonDown'] and not i:
                    #print "SumVelocity",dstate['sumVelocity']
                    if dstate['newupdate']:
                        if 'sumVelocity' not in dstate:
                            dstate['sumVelocity'] = [0,0,0]
                            dstate['countVelocity'] = 0
                        dstate['sumVelocity'] = vectorops.add(dstate['sumVelocity'],dstate['linearVel'])
                        dstate['countVelocity'] += 1
                        time_now = time.time()
                        if time_now - self.basePosCommandTimer[i] > 0.1:
                            averageVelocity = vectorops.div(dstate['sumVelocity'],dstate['countVelocity'])
                            #print "Average velocity",averageVelocity
                            for i in range(0,2):
                                if averageVelocity[i] > 0.01 or averageVelocity[i] < -0.01:
                                    baseVel[i] = -averageVelocity[i]*self.basePosVelocityRatio/5
                                else:
                                    baseVel[i] = 0.0
                            msg['type'] = 'BaseVelocity'
                            msg['velocity'] = baseVelocityViewToWorld(baseVel)
                            # msg['velocity'] = [v * self.baseVelocityRatio for v in baseVel]
                            self.basePosControlFlag = True
                            self.gripperSizeFlag = False
                            self.baseDevice = True
                            self.basePosCommandTimer[i] = time.time()
                            dstate['sumVelocity'] = [0,0,0]
                            dstate['countVelocity'] = 0
                    else:
                        self.basePosControlFlag = True
                        msg['type'] = 'BaseVelocity'
                        msg['velocity'] = [0, 0, 0]

        self.baseOrtControlFlag = False
        for i, dstate in enumerate(deviceState):
            if dstate['mode'] not in ['baseOrientation', 'base']: continue
            # if i == 0:
            if dstate['buttonDown'] and i:
                if dstate['newupdate']:
                    if 'sumVelocity' not in dstate:
                        dstate['sumVelocity'] = [0,0,0]
                        dstate['countVelocity'] = 0
                    dstate['sumVelocity'] = vectorops.add(dstate['sumVelocity'],dstate['linearVel'])
                    dstate['countVelocity'] += 1
                    time_now = time.time()
                    if time_now - self.baseOrtCommandTimer[i] > 0.1:
                        averageVelocity = vectorops.div(dstate['sumVelocity'],dstate['countVelocity'])
                        if averageVelocity[1] > 0.01:
                            baseVel[2] = averageVelocity[1]/5
                        elif averageVelocity[1] < -0.01:
                            baseVel[2] = averageVelocity[1]/5
                            # baseVel[2] = dstate['linearVel'][1]*self.baseOrtVelocityRatio
                        else:
                            baseVel[2] = 0.0
                        msg['type'] = 'BaseVelocity'
                        msg['velocity'] = baseVel
                        # msg['velocity'] = [v * self.baseVelocityRatio for v in baseVel]
                        self.baseOrtControlFlag = True
                        self.basePosControlFlag = False
                        self.gripperSizeFlag = False
                        self.baseOrtCommandTimer[i] = time.time()
                        dstate['sumVelocity'] = [0,0,0]
                        dstate['countVelocity'] = 0
                else:
                    self.baseOrtControlFlag = True
                    msg['type'] = 'BaseVelocity'
                    msg['velocity'] = [0,0,0]

        if self.basePosControlFlag or self.baseOrtControlFlag:
            return msg
        else:
            return None

    def renderCartesianVelocityMsg(self):
        """DEPRECATED"""
        deviceState = self.serviceThread.hapticupdater.deviceState
        if len(deviceState)==0:
            print "No device state"
            return None
        msg = {}
        msg['type'] = 'CartesianVelocity'
        msg['limb'] = 'both'
        msg['linear'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg['angular'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg['safe'] = 0

        for i,dstate in enumerate(deviceState):
            if dstate['buttonDown']:
                if dstate['newupdate']:
                    print "Device",i,"linear",dstate['linearVel'],"angular",dstate['angularVel']
                    msg['linear'][0+i*3:3+i*3] = dstate['linearVel']
                    msg['angular'][0+i*3:3+i*3] = dstate['angularVel']
                    dstate['newupdate'] = False
        return msg

    def renderCartesianPoseMsg(self):
        deviceState = self.serviceThread.hapticupdater.deviceState
        if len(deviceState)==0:
            print "No device state"
            return None
        transforms = [self.serviceThread.eeGetter_left.get(),self.serviceThread.eeGetter_right.get()]
        update = [False,False]
        for i,dstate in enumerate(deviceState):
            if dstate['mode'] not in ['arm', 'normal', 'onlyPosition'] : continue
            if dstate['buttonDown']:
                if self.startTransforms[i] == None:
                    self.startTransforms[i] = transforms[i]
                    #RESET THE HAPTIC FORCE FEEDBACK CENTER
                    self.serviceThread.hapticupdater.activateSpring(kP=20,kD=0,center=None)
                    #print "Start transform",transforms[i]
                if dstate['newupdate']:
                    update[i] = True
                    dstate['newupdate'] = False
            else:
                self.startTransforms[i] = None

        if any(update):
            msg = {}
            msg['type'] = 'CartesianPose'
            msg['limb'] = 'both' if all(update) else SIDES[update[1]] ## benefits of narrowing down to XOR

            R1,t1 = self.getDesiredCartesianPose('left',0) if update[0] else ([],[])
            R2,t2 =  self.getDesiredCartesianPose('right',1) if update[1] else ([],[])
            transforms = [(R1,t1),(R2,t2)]
            msg['position'] = t1+t2
            msg['rotation'] = R1+R2
            msg['maxJointDeviation']=0.3
            msg['safe'] = int(CollisionDetectionEnabled)
            self.CartesianPoseControl = True
            return msg
        else:
            return None

    def cameraSwitch(self):
        deviceState = self.serviceThread.hapticupdater.deviceState
        if deviceState[0]['buttonDown'] and not self.last_buttonDown[0]:
            print "switching secondary main camera view"
            self.switch_pub.publish("switch  secondary")
        if deviceState[1]['buttonDown'] and not self.last_buttonDown[1]:
            print "switching main camera view"
            self.switch_pub.publish("switch")

        self.last_buttonDown = [deviceState[0]['buttonDown'], deviceState[1]['buttonDown']]

    def getRobotStatus(self):
        T1 = self.serviceThread.eeGetter_left.get()
        if T1 is  None:
            T1 = se3.identity()
        R1,t1=T1
        T2 = self.serviceThread.eeGetter_right.get()
        if T2 is  None:
            T2 = se3.identity()
        R2,t2=T2
        gripperStatus_left = self.serviceThread.gripperGetter_left.get()
        gripperStatus_right = self.serviceThread.gripperGetter_right.get()

        self.baseSensedVelocity = self.serviceThread.baseVelocitySensed.get()
        self.baseCommandVelocity = self.serviceThread.baseVelocityCommand.get()

        self.ArmPosition[0] = self.serviceThread.ArmPosition_left.get()
        self.ArmPosition[1] = self.serviceThread.ArmPosition_right.get()

        self.robotEndEffectorPosition[0] = t1
        self.robotEndEffectorPosition[1] = t2

        self.robotEndEffectorTransform[0] = T2
        self.robotEndEffectorTransform[1] = T2

        # self.gripperPosition[0] = gripperStatus_left
        # self.gripperPosition[1] = gripperStatus_right

        # for i in range(0,2):
        #     if self.gripperPosition[i]:
        #         if all(v > 0.9 for v in self.gripperPosition[i]):
        #             self.gripperStatus[i] = "open"
        #         elif all(v < 0.3 for v in self.gripperPosition[i]):
        #             self.gripperStatus[i] = "close"

    def moveToPreset(self):
        pass
        # vals = self.plugin.limb_pose.values()
        # if any(vals):
        #     msg = {}
        #     msg['type'] = "JointPose"
        #     msg['part'] = 'both' if all(vals) else SIDES[vals[1]]
        #     # get preset values for each side, empty if the limb is free
        #     msg['position'] = alignArmPose[self.plugin.limb_pose['left'].value][0] + \
        #                       alignArmPose[self.plugin.limb_pose['right'].value][1]
        #     msg['speed'] = 1
        #     msg['safe'] = int(CollisionDetectionEnabled)
        #     self.plugin.limb_pose['left'] = LimbPose.FREE
        #     self.plugin.limb_pose['right'] = LimbPose.FREE
        #     return msg

    def do_logic(self):
        # Preset pose buttons. This is unilateral and could be combined if the task submissions are alternated or
        # if they both used the joint poses
        # self.moveToPreset()
        poses = [pose.value for pose in self.plugin.limb_pose.values()]
        if any(poses):
            msg = {}
            msg['type'] = "JointPose"
            msg['parts'] = ['left','right'] if all(poses) else [SIDES[bool(poses[0])]] # there is a better way to do this but it escapes me atm
            # get preset values for each side, empty if the limb is free
            print(alignArmPose[self.plugin.limb_pose['left']])
            print(alignArmPose[self.plugin.limb_pose['right']])

            msg['positions'] = [alignArmPose[self.plugin.limb_pose[side]][side] for side in msg['parts']]
            msg['speed'] = 1
            msg['safe'] = int(CollisionDetectionEnabled)
            self.plugin.limb_pose['left'] = LimbPose.FREE
            self.plugin.limb_pose['right'] = LimbPose.FREE

            return msg
        else:
            self.plugin.limb_pose['left'] = LimbPose.FREE
            self.plugin.limb_pose['right'] = LimbPose.FREE

        if HapticMode == 'arm':
            msg = self.renderCartesianPoseMsg()
            if self.CartesianPoseControl:
                self.CartesianPoseControl = False
                return msg

        elif HapticMode == 'base':
            msg = self.BasePositionControl()
            if self.basePosControlFlag or self.baseOrtControlFlag:
                self.basePosControlFlag = False
                self.baseOrtControlFlag = False
                return msg

        elif HapticMode == 'gripper':
            self.gripperPinchGrasp(1.0)
            return {}
        elif HapticMode == 'gripper_pinch':
            self.gripperPinchGrasp(0.7)
            return {}
        elif HapticMode == 'camera':
            self.cameraSwitch()
            return {}
        else:
            return {}

    def glPlugin(self):
        return self.plugin

def make():
    return HapticTaskGenerator()
