from nokov.nokovsdk import *
import time
import sys, getopt

import rclpy
from rclpy.node import Node
from ros2_interfaces.msg import Motions
from geometry_msgs.msg import Pose2D, Vector3, Point, Twist
from collections import deque
from math import atan2
from typing import List
import copy

from math import radians
from rmcontrol.utils import get_ori



preFrmNo=0
curFrmNo = 0
pi=3.14159265
no_robots=12

class MotionCapture(Node):
    def __init__(self,name):
        super().__init__(name)
        serverIp = '10.1.1.198'
        print ('serverIp is %s' % serverIp)
        client = PySDKClient()

        ver = client.PySeekerVersion()
        print('SeekerSDK Sample Client 2.4.0.3142(SeekerSDK ver. %d.%d.%d.%d)' % (ver[0], ver[1], ver[2], ver[3]))

        client.PySetVerbosityLevel(0)
        client.PySetMessageCallback(self.py_msg_func)
        client.PySetDataCallback(self.py_data_func, None)

        print("Begin to init the SDK Client")
        ret=1
        while ret:
            ret = client.Initialize(bytes(serverIp, encoding = "utf8"))

            if ret == 0:
                print("Connect to the Seeker Succeed")
            else:
                print("Connect Failed: [%d]" % ret)
                # exit(0)
                time.sleep(2)

        #Give 5 seconds to system to init forceplate device
        ret = client.PyWaitForForcePlateInit(5000)
        if (ret != 0):
            print("Init ForcePlate Failed[%d]" % ret)
            exit(0)

        client.PySetForcePlateCallback(self.py_forcePlate_func, None)
        
        self.mo_pub = self.create_publisher(Motions,"motions", 10)
        self.p = []
        self.v = []
        self.tdq=deque()
        self.pdq=deque()
        self.theta_dq=deque()
        print('motion capture initialization finished!!')
        time.sleep(2)
        while True:
            self.publisher()


    def publisher(self):
        self.mo_pub.publish(Motions(poses=self.p, twists=self.v))
        # self.get_logger().info(f'当前位置：{self.poses}')
        # self.get_logger().info(f'当前速度：{self.twists}')


    def py_data_func(self, pFrameOfMocapData, pUserData):
        if pFrameOfMocapData == None:  
            print("Not get the data frame.\n")
        else:
            frameData = pFrameOfMocapData.contents
            global preFrmNo, curFrmNo 
            curFrmNo = frameData.iFrame
            if curFrmNo == preFrmNo:
                return

            preFrmNo = curFrmNo
            n_o=frameData.nOtherMarkers
            self.p=[]
            self.v=[]
            if n_o not in [1,2,3]:
                return 
            else:
                pose_ball=Pose2D(x=frameData.OtherMarkers[0][0]/1000,
                                 y=frameData.OtherMarkers[0][1]/1000)
                self.p.append(pose_ball)
            # else:
            #     pose_ball=Pose2D(x=frameData.OtherMarkers[0][0]/1000,
            #                      y=frameData.OtherMarkers[0][1]/1000)
            #     self.poses.pose.append(pose_ball)

            for iBody in range(frameData.nRigidBodies):
                body = frameData.RigidBodies[iBody]
                self.p.append(Pose2D(x=body.x/1000, y=body.y/1000, theta=get_ori(body.qz, body.qw)))

            self.tdq.append(time.time())
            self.pdq.append(copy.deepcopy(self.p))

            if len(self.tdq)>2 and len(self.pdq)>2:
                self.tdq.popleft()
                self.pdq.popleft()
                dt=self.tdq[1]-self.tdq[0]
                if dt!=0:
                    for i in range(no_robots+1):
                        x=(self.pdq[1][i].x-self.pdq[0][i].x)/dt
                        y=(self.pdq[1][i].y-self.pdq[0][i].y)/dt
                        z=(self.pdq[1][i].theta-self.pdq[0][i].theta)/dt
                        self.v.append(Pose2D(x=x/1000, y=y/1000, theta=z))

                


    def py_msg_func(iLogLevel, szLogMessage):
        szLevel = "None"
        if iLogLevel == 4:
            szLevel = "Debug"
        elif iLogLevel == 3:
            szLevel = "Info"
        elif iLogLevel == 2:
            szLevel = "Warning"
        elif iLogLevel == 1:
            szLevel = "Error"
    
        print("[%s] %s" % (szLevel, cast(szLogMessage, c_char_p).value))

    def py_forcePlate_func(pFocePlates, pUserData):
        if pFocePlates == None:  
            print("Not get the forcePlate frame.\n")
            pass
        else:
            ForcePlatesData = pFocePlates.contents
            print("iFrame:%d" % ForcePlatesData.iFrame)
            for iForcePlate in range(ForcePlatesData.nForcePlates):
                print("Fxyz:[%f,%f,%f] xyz:[%f,%f,%f] MFree:[%f]" % (
                    ForcePlatesData.ForcePlates[iForcePlate].Fxyz[0],
                    ForcePlatesData.ForcePlates[iForcePlate].Fxyz[1],
                    ForcePlatesData.ForcePlates[iForcePlate].Fxyz[2],
                    ForcePlatesData.ForcePlates[iForcePlate].xyz[0],
                    ForcePlatesData.ForcePlates[iForcePlate].xyz[1],
                    ForcePlatesData.ForcePlates[iForcePlate].xyz[2],
                    ForcePlatesData.ForcePlates[iForcePlate].Mfree
                ))


def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = MotionCapture("motion_capture")  # 新建一个节点
    rclpy.spin_once(node) 
    node.destroy_node()
    rclpy.shutdown() # 关闭rclpy



