from typing import Tuple

import rtde_control
import rtde_receive
from Motor_Code import Motors
import UR5_Interface as ur
import RealSense as real
import ObjectDetection as ob
import TaskPlanner as tp
import Block as bl
import open3d as o3d
import numpy as np
import serial.tools.list_ports

try:
    robotIP = "192.168.0.6"
    con = rtde_control.RTDEControlInterface(robotIP)
    rec = rtde_receive.RTDEReceiveInterface(robotIP)
    servoPort = "/dev/ttyACM0"
    # servoPort = get_USB_port_with_desc("OpenRB") # this and the method are from Magpie
    gripperController = Motors(servoPort)
    print("we made it here")
    gripperController.torquelimit(600)  # used to be 600
    gripperController.speedlimit(100)
    ur = ur.UR5_Interface()
    ur.gripperController = gripperController
    try:
        ur.c = con
        ur.r = rec
        ur.gripperController = gripperController
    except Exception as e:
        raise (e)
    else:
        print("UR5 + Gripper Interface Established")
    real = real.RealSense()
    real.initConnection()
    try:
        detector = ob.ObjectDetection(real, None, moveRelative=True)
        print("Detector established")
    except Exception as e:
        detector.real.pipe.stop()
        raise (e)
    urPose = ur.getPose()
    jointAngles = ur.getJointAngles()
    print("Joint Angles: ", jointAngles * 180 / np.pi)
    # print("p pose: ", p) # p comes from the variable in the getPose method, I needed this to get the correct setup for the robot position
except Exception as e:
    gripperController.disconnect()
    ur.c.disconnect()
    ur.r.disconnect()
    real.disconnect()
    raise (e)
