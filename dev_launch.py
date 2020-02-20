import subprocess
import signal


rs_head = subprocess.Popen(["roslaunch", "ebolabot_bringup", "usb_cam.launch", "dev_rsl:=/dev/video9", "res:=480", "aspect_factor:=12", "name:=rs_head"])
rs_right = subprocess.Popen(["roslaunch", "ebolabot_bringup", "usb_cam.launch", "dev_rsl:=/dev/video2", "res:=480", "aspect_factor:=12", "name:=rs_right"])
rs_left = subprocess.Popen(["roslaunch", "ebolabot_bringup", "usb_cam.launch", "dev_rsl:=/dev/video6", "res:=480", "aspect_factor:=12", "name:=rs_left"])

cam_ctrl = subprocess.Popen(["python", "cam_base_ctrl.py"])

try:
    rs_head.wait()
except KeyboardInterrupt:
    print "Abort!"
    rs_head.send_signal(signal.SIGINT)
    rs_right.send_signal(signal.SIGINT)
    rs_left.send_signal(signal.SIGINT)
    cam_ctrl.send_signal(signal.SIGINT)
