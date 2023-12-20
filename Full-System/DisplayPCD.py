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

# use GetRobotPose.py to get the position matrix for where I manually moved the robot to
pos_left = np.array([-0.05591429128391958, -0.5380791072811699, 0.02990208142149539, 1.8719105457391148, -0.36717143076244796, -0.28374441118214333])

# TODO: the robot moves way too fast, from pos_left to pos_right. I need to figure out how to slow this down
pos_right = np.array([-0.13941630966009855, -0.5429747311181137, 0.056392101609168285, 1.89720228366065, 0.20627407259726946, 0.09467935487757431])


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
        o3d.visualization.draw_geometries([pcd])

    o3d.visualization.draw_geometries(pcds)

    merged_point_cloud = o3d.geometry.PointCloud() # creates an empty pcd object

    # TODO: Question 1. Is this even possible? If this is possible, we don't need code changes. I don't think this is possible
    for pc in pcds:
        merged_point_cloud += pc

    return pcds, rgbds


try:
    robotIP = "192.168.0.6"
    con = rtde_control.RTDEControlInterface(robotIP)
    rec = rtde_receive.RTDEReceiveInterface(robotIP)
    servoPort = "/dev/ttyACM0"
    # servoPort = get_USB_port_with_desc("OpenRB") # this and the method are from Magpie
    gripperController = Motors(servoPort)
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

    # switch get_pcd_at_multiple_positions method to return list of pcds
    # use displayPCD from RealSense class

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
