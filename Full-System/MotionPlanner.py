class MotionPlanner():
    def __init__(self, blocks, moveRelative=True):
        # :moveRelative boolean - True if the gripper moves to block positions in the gripper frame, false if moving to world frame positions (via rtb_model)
        # self.robot_model = RTB_Model()
        self.blocks = blocks
        self.moveRelative = moveRelative

    def displayOpen3D(self):
        pass

    def displaySwift(self):
        self.robot_model.initSwiftEnv()
        for Block in self.blocks:
            self.robot_model.addSwiftBox(pos=Block.blockAABB.get_center())

    def runMovement(self):
        # self.ur,self.rtb_model,self.blocks
        blueBlock = self.blocks[0]
        yellowBlock = self.blocks[1]
        xB, yB, zB = yellowBlock.gripperFrameCoords  # grasp point in gripper frame
        print(f"Block Coordinate:({xB * 1000},{yB * 1000},{zB * 1000})")
        currentPose = self.ur.getPose()  # SE3 Object
        R = currentPose.R
        pX, pY, pZ = tuple(currentPose.t)
        print(f"Current Pose:\n{currentPose * 1000}")
        print(f"Pose Coordinate: ({pX * 1000},{pY * 1000},{pZ * 1000})")
        if self.moveRelative == False:
            # Move directly to block position in world frame
            goalPose = copy.deepcopy(currentPose)
            goalPose.t[0] = xB
            goalPose.t[1] = yB
        else:
            # Move relative to current position given block position in gripper frame
            # subtract 0.175 from block position to account for gripper length
            zB -= 0.165
            pX, pY, pZ = np.array(currentPose.t)
            print(f"pZ:{pZ}")
            # xB,yB,zB here is the block position in the gripper frame which is aligned with the optoforce frame
            R = self.ur.getPose().R
            P_goal = np.matmul(R, np.array([xB, yB, zB]).T)  # relative position of the block in world coordinates
            goalX, goalY, goalZ = tuple(
                P_goal)  # quantities and directions the the gripper frame should be incremented to be centered at the block
            goalPose = copy.deepcopy(currentPose)  # maintain rotation and shift position
            print(f"P_goal:\n{P_goal}")
            goalPose.t[0] += goalX
            goalPose.t[1] += goalY

        print(f"Goal Coordinate ({goalPose.t[0] * 1000},{goalPose.t[1] * 1000},{goalPose.t[2] * 1000})")
        print("Moving to goal")
        print(f"Goal Pose\n {goalPose}")
        # self.ur.moveL(goalPose)
        goalPose.t[2] += goalZ
        # self.ur.moveL(goalPose)
        # self.ur.closeGripper(18)


'''
detector = ObjectDetection()

try:
    pcd,rgbdImage = detector.real.getPCD()
    # rgbdImage = detector.real.takeImages()
    depthImage,colorImage = rgbdImage.depth,rgbdImage.color
    detector.real.displayImages(depthImage,colorImage)
    redPCD,yellowPCD,bluePCD = detector.colorSegmentation(colorImage,depthImage)
    # detector.real.displayPCD([pcd])
    detector.real.displayPCD([redPCD,yellowPCD,bluePCD])
    blocks = [Block("Red Block",redPCD),Block("Yellow Block",yellowPCD),Block("Blue Block",bluePCD)]
    items = []
    for block in blocks:
        s = o3d.geometry.TriangleMesh.create_sphere(radius=0.0125/4)
        s.translate(block.getGraspPoint())
        items.extend([block.blockPCD,block.blockAABB,s])
    detector.real.displayPCD(items)
    m = MotionPlanner(blocks)
    m.display()

    # o3d.io.write_point_cloud("redPCD.pcd",redPCD)
except Exception as e:
    raise(e)
finally:
    detector.real.pipe.stop()
'''