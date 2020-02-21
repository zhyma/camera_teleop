import os
import subprocess
import signal

rs_serials = {
    'head':  '816513020817',
    'right': '908323050243',
    'left':  '824613021062'
}

def search4dev(devices, cam):
    # Search in a reversed manner. Because RGB is the last subdevice.
    for i in devices:
        cmd = subprocess.Popen(['udevadm', 'info', '--query=all', '/dev/' + i], stdout=subprocess.PIPE)
        try:
            grep = subprocess.check_output(('grep', 'SERIAL_SHORT'), stdin=cmd.stdout)
        except subprocess.CalledProcessError as e:
            pass
        if rs_serials[cam] in grep:
            print 'Find device ' + i
            return i

    print 'ERROR: cannot find RealSense: ' + cam
    return ''
    
def devBringUp(dev):
    cmd1 = ['roslaunch', 'ebolabot_bringup', 'usb_cam.launch']
    cmd2 = ['res:=480', 'aspect_factor:=12']
    return subprocess.Popen(cmd1 + ['dev_rsl:=/dev/'+dev[1]] + cmd2 + ['name:='+dev[0]])
    

if __name__ == '__main__':
    video_det = subprocess.Popen(['ls', '/dev'], stdout=subprocess.PIPE)
    dev_list = []
    for i in video_det.stdout.readlines():
        if i[0]=='v' and 'video' in i:
            # The last char in i is '\n'
            dev_list.append(i[:-1])

    dev_list.reverse()
    rs_head_name = search4dev(dev_list, 'head')
    rs_right_name = search4dev(dev_list, 'right')
    rs_left_name = search4dev(dev_list, 'left')

    print rs_head_name
    print rs_right_name
    print rs_left_name

    if rs_head_name == '' or rs_right_name == '' or rs_left_name == '':
        print 'ERROR: Missing devices!'
        exit()

    rs_head  = devBringUp(['rs_head', rs_head_name])
    rs_right = devBringUp(['rs_right', rs_right_name])
    rs_left  = devBringUp(['rs_left', rs_left_name])
    
    gripper = subprocess.Popen(['roslaunch', 'gentle_ros', 'gentle_ros_launcher1.launch'])

    cam_ctrl = subprocess.Popen(['python', 'cam_base_ctrl.py'])

    try:
        rs_head.wait()
    except KeyboardInterrupt:
        print 'Abort!'
        rs_head.send_signal(signal.SIGINT)
        print 'quit rs_head'
        rs_right.send_signal(signal.SIGINT)
        print 'quit rs_right'
        rs_left.send_signal(signal.SIGINT)
        print 'quit rs_left'
        gripper.send_signal(signal.SIGINT)
        print 'quit gripper'
        cam_ctrl.send_signal(signal.SIGINT)
        print 'quit cam_ctrl'
        
