# A camera teleoperation package for Trina.

Package name: camera_teleop

The "motorized_cam_ctrl" folder contains the source code for the OpenCM 9.04. You need to have the software prepared to flash your OpenCM board.

The servos are named as ID=1 for yaw, ID=2 for pitch. The TTL control command is "0x80, pos1, speed1, pos2, speed2, 0x7F". We use 0x80 and 0x7F for start and end byte because they are not used for position mode (+/-180deg). The second byte is the position of servo ID=1, measured in degree, int type. 0 for neutral, left/CCW is set to positive, right/CW is set to negtive. The fourth byte is the position of servo ID=2

## Task_Gen

Files under Task_Gen should be moved to your iml-internal/Ebolabot. For the TaskGUI file (those *.py under TaskGenerators), you can use command

'''
$ ln -s your_folder_location/Task_Gen/TaskGenerators/vive_cam_hc.py ./
'''

to create a soft link to your file. Then you don't need to copy and paste it back and forth after editing.
