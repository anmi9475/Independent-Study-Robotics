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


# def get_USB_port_with_desc(descStr):
#     match = None
#     for port, desc, hwid in sorted (serial.tools.list_ports.comports()):
#         if descStr in desc:
#             match = port
#             break
#     return match


pos_left = np.array([76.8, -91.67, 138.5, -33.3, 95.3, -3.0])
pos_right = np.array([17.50, -86.5, 134.1, -10.13, 22.3, -38.3])
# TODO: modify this as needed to get into position


positions_array = [
    pos_left,
    pos_right,
]


def get_pcd_at_multiple_positions(robot: ur.UR5_Interface, camera: real.RealSense) -> Tuple[any, list]:
    pcds = []
    rgbds = []
    for pos in positions_array:
        print("pos: ", pos)
        robot.moveToPosition(pos)
        pcd, rgbd = camera.getPCD()
        pcds.append(pcd)
        rgbds.append(ob.TorchImage(robot.getPose(), rgbd.color, rgbd.depth))

    o3d.visualization.draw_geometries(pcds)

    merged_point_cloud = o3d.geometry.PointCloud() # creates an empty pcd object

    for pc in pcds:
        merged_point_cloud += pc

    return merged_point_cloud, rgbds


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
    # pcd, rgbdImage = detector.real.getPCD()  # HERE
    pcd, rgbds = get_pcd_at_multiple_positions(ur, real)

    blocks = detector.getBlocksFromImages(rgbds)


    # need a method to get the best set of blocks from blocksList



    planner = tp.TaskPlanner(blocks)
    # goalDict = {"on":[("blueBlock", "on":[("redBlock","yellowBlock")])]}
    goalDict = {"on": [("redBlock", "yellowBlock")]}
    steps = planner.generatePlan(goalDict)
    print(steps)
    for block in blocks:
        print(f"{block.name} - {list(block.gripperFrameCoords)}")
    detector.displayWorld(pcd, blocks)
    gripperController.disconnect()
    ur.c.disconnect()
    ur.r.disconnect()
    real.disconnect()
except Exception as e:
    gripperController.disconnect()
    ur.c.disconnect()
    ur.r.disconnect()
    real.disconnect()
    raise (e)
