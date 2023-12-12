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

# TODO: modify this as needed to get into position
pos_left = np.array([[0.99955322, -0.02418213, -0.01756664, 0.01498893],
                     [-0.01748495, 0.00358545, -0.9998407, -0.57686779],
                     [0.02424126, 0.99970114, 0.00316103, 0.05545535],
                     [0, 0, 0, 1]])
# TODO: modify this as needed to get into position
pos_right = np.array([[0.99955322, -0.02418213, -0.01756664, 0.01498893],
                      [-0.01748495, 0.00358545, -0.9998407, -0.57686779],
                      [0.02424126, 0.99970114, 0.00316103, 0.05545535],
                      [0, 0, 0, 1]])

positions_array = [
    pos_left,
    pos_right,
]


def get_pcd_at_multiple_positions(robot: ur.UR5_Interface, camera: real.RealSense) -> Tuple[any, list]:
    pcds = []
    rgbds = []
    for pos in positions_array:
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
    except Exception as e:
        detector.real.pipe.stop()
        raise (e)
    urPose = ur.getPose()
    jointAngles = ur.getJointAngles()
    print("Joint Angles: ", jointAngles * 180 / np.pi)
    # pcd, rgbdImage = detector.real.getPCD()  # HERE
    pcd, rgbds = get_pcd_at_multiple_positions(ur, real)

    blocks = detector.getBlocksFromImages(colorImage, depthImage, urPose)


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
