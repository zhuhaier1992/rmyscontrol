import rclpy
from rclpy.node import Node
from ros2_interfaces.msg import Motions, Ctrl
from ros2_interfaces.srv import AgentStatus
from geometry_msgs.msg import Twist, Vector3, Pose2D
import time
from .utils import *
from .RVO import RVO_update, compute_V_des
import numpy as np


no_rm=6
no_ys=6

class Rvo(Node):
    def __init__(self, name):
        super().__init__(name)
        self.name=name
        self.mo_sub=self.create_subscription(Motions, 'motions', self.update_pv, 10)
        self.ctrl_sub=self.create_subscription(Ctrl, 'to_rvo', self.calc_v, 10)
        self.rvo_pub=self.create_publisher(Ctrl, 'rvo', 10)
        self.p=np.zeros([no_rm+no_ys+1,3])
        self.v=np.zeros([no_rm+1,3])
        self.tp=np.zeros([no_rm+1,3])


    def update_pv(self, msg):
        if len(msg.poses)>12:
            self.p=pose2d_to_nparray(msg.poses)
            self.v=pose2d_to_nparray(msg.twists)


    def calc_v(self, msg):
        # if len(self.p)>no_rm:
        c=msg.code
        self.tp=pose2d_to_nparray(msg.pose)
        self.tp=np.vstack([self.tp, self.p[7:13]])
        v_max = [1.2]*(no_rm+no_ys)
        ws_model = {'robot_radius': 0.15, 'circular_obstacles': []}
        # self.get_logger().info(f'p: {self.p[1:no_rm+1]}')
        v_des = compute_V_des(self.p[1:no_rm+no_ys+1], self.tp[1:no_rm+no_ys+1], v_max)
        v_best = RVO_update(self.p[1:no_rm+no_ys+1], v_des, self.v[1:no_rm+no_ys+1], ws_model)
        rel_v=np.zeros([no_rm+1,3])
        
        for i in range(1, no_rm+1):
            # if distance(self.p[i], self.tp[i])>0.5:
            #     if norm(v_best[i-1])<0.1:
            #         v_best[i-1]=v_best[i-1]/norm(v_best[i-1])*0.1
            # self.get_logger().info(f'{i}: v_best:{v_best[i-1]}, tp: {self.tp[i][2]}')
        
            rel_v[i]=trans_relative_co(np.array([0,0,self.p[i][2]]), 
                            np.hstack((v_best[i-1],0)))
            rel_v[i][2]=limit_pi(self.tp[i][2]-self.p[i][2])
            
        self.rvo_pub.publish(Ctrl(code=c, pose=nparray_to_pose2d(rel_v)))
        




def main(args=None):
    rclpy.init(args=args)
    node = Rvo(name='rvo')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()