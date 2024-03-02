from nokov.nokovsdk import *
import time
import sys, getopt

import rclpy
from rclpy.node import Node
from ros2_interfaces.msg import Motions, Mocap
from geometry_msgs.msg import Pose2D, Vector3, Point, Twist
from collections import deque
from math import atan2
from typing import List
import copy

from math import radians, sqrt
from rmcontrol.utils import get_ori



preFrmNo=0
curFrmNo = 0
pi=3.14159265
no_robots=12

def get_norm(x,y):
    return sqrt(x**2+y**2)

def distance(x,y):
    return sqrt((x[0]-y[0])**2+(x[1]-y[1])**2)

class MotionCapture(Node):
    def __init__(self,name):
        super().__init__(name)
        self.pdq=deque()
        serverIp = '10.1.1.198'
        print ('serverIp is %s' % serverIp)
        ret=1
        while ret:
            client = PySDKClient()
            ver = client.PySeekerVersion()
            print('SeekerSDK Sample Client 2.4.0.3142(SeekerSDK ver. %d.%d.%d.%d)' % (ver[0], ver[1], ver[2], ver[3]))
            client.PySetVerbosityLevel(0)
            client.PySetMessageCallback(self.py_msg_func)
            client.PySetDataCallback(self.py_data_func, None)
            print("Begin to init the SDK Client")
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
        self.stop_pub=self.create_publisher(Mocap, 'mocap',1)
        self.error_cnt=0
        self.p = []
        self.v = []
        self.tdq=deque()
        
        self.theta_dq=deque()
        self.last_ball=Pose2D()
        self.verbose=False
        print('motion capture initialization finished!!')
        time.sleep(2)
        while True:
            self.publisher()


    def publisher(self):
        self.mo_pub.publish(Motions(poses=self.p, twists=self.v))
        # self.get_logger().info(f'当前位置：{self.poses}')
        # self.get_logger().info(f'当前速度：{self.twists}')


    def py_data_func(self, pFrameOfMocapData, pUserData):
        if self.pdq==None:
            self.pdq=deque()
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
            self.temp_p=[0]
            self.v=[]
            p_o=[]
            if n_o <1:
                return 
            else:
                pose_ball=Pose2D(x=frameData.OtherMarkers[0][0]/1000,
                                y=frameData.OtherMarkers[0][1]/1000)
                
                self.temp_p[0]=pose_ball
                    # dt=self.tdq[1]-self.tdq[0]
                    # vmin=100
                    # for i in range(n_o):
                    #     pi=[frameData.OtherMarkers[i][0]/1000, frameData.OtherMarkers[i][1]/1000]
                    #     p_o.append(pi)
                        
                    #     dx=self.pdq[0][0].x-pi[0]
                    #     bvx=dx/dt
                    #     dy=self.pdq[0][0].y-pi[1]
                    #     bvy=dy/dt
                    #     n=get_norm(bvx, bvy)
                    #     if self.verbose:
                    #         self.get_logger().warn(f'speed: {bvx, bvy}, dx: {dx}, dt: {dt}')
                    #     if n<vmin:
                    #         if self.verbose:
                    #             self.get_logger().info(f'good ball speed: {bvx, bvy}')
                    #         vmin=n
                    #         pose_ball=Pose2D(x=pi[0],y=pi[1])
                    #         self.temp_p[0]=pose_ball
                    #         break
                    #     elif distance(pi, [self.last_ball.x, self.last_ball.y])<0.3:
                    #         if self.verbose:
                    #             self.get_logger().info(f'distance: {distance(pi, [self.last_ball.x, self.last_ball.y])}')
                    #         self.temp_p=Pose2D(x=pi[0],y=pi[1])
                self.p=self.temp_p.copy()

            
                self.stop_pub.publish(Mocap(stop=0))
                self.verbose=False
                self.p=self.temp_p.copy()
                self.last_ball=self.p[0]

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
                        try: 
                            for i in range(no_robots+1):
                                
                                x=(self.pdq[1][i].x-self.pdq[0][i].x)/dt
                                y=(self.pdq[1][i].y-self.pdq[0][i].y)/dt
                                z=(self.pdq[1][i].theta-self.pdq[0][i].theta)/dt
                                self.v.append(Pose2D(x=x/1000, y=y/1000, theta=z))
                        except Exception as e:
                            # self.get_logger().error(e)
                            self.get_logger().info(f'pdq: {len(self.pdq)}')

                


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



