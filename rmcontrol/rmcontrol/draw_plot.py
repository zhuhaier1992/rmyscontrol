import rclpy
from rclpy.node import Node
from ros2_interfaces.msg import Motions, Ctrl
from ros2_interfaces.srv import AgentStatus
from geometry_msgs.msg import Twist, Vector3, Pose2D
import time
from .utils import *
from .RVO import RVO_update, compute_V_des
import numpy as np
from matplotlib import pyplot as plt
import re


no_rm=6

class Draw(Node):
    def __init__(self, name):
        super().__init__(name)
        self.name=name
        self.mo_sub=self.create_subscription(Motions, 'motions', self.update_pv, 10)
        self.ctrl_sub=self.create_subscription(Ctrl, 'to_rvo', self.calc_v, 10)
        self.rvo_pub=self.create_publisher(Ctrl, 'rvo', 10)
        self.p=np.zeros([no_rm+1,3])
        self.v=np.zeros([no_rm+1,3])
        self.tp=np.zeros([no_rm+1,3])
        self.code=[]

    def update_pv(self, msg):
        if len(msg.poses)!=0:
            self.p=pose2d_to_nparray(msg.poses)
            self.v=pose2d_to_nparray(msg.twists)

    def calc_v(self, msg):
        self.code=msg.code
        self.tp=pose2d_to_nparray(msg.pose)

def main(args=None):
    rclpy.init(args=args)
    node = Draw(name='rvo')
    fig1=plt.figure('frame', facecolor='black')
    while rclpy.ok():
        rclpy.spin_once(node)
        # pb=node.p[0]
        # t_min=1000.
        # p0=np.array([0,0])
        # id=0
        # ts={}
        # for i in range(1, len(node.p)):
        #     t, p0, solutions=reach_time(node.p[i], pb, node.v[0])
        #     node.get_logger().info(f'solutions: {solutions}, t: {t}, p0: {p0}')
            
        #     calc_t=distance(node.p[i], pb)/0.5
        #     node.get_logger().info(f'i: {i}, p: {node.p[i]}, vb: {node.v[0]}, calc_t: {calc_t}')
        #     # plt.scatter(p0[0], p0[1], c='dimgray', s=500, marker='*')
        #     ts[i]=t
        #     if 0< t < t_min:
        #         t_min, id=t, i
        
        # node.get_logger().info(f'kicker: {id}, ts: {ts}')
        plt.scatter(node.p[0, 0], node.p[0, 1], c='black', s=400, marker='o')
        plt.scatter(node.p[1:4, 0], node.p[1:4, 1], c='red', s=800, marker='s')
        plt.scatter(node.p[4:7, 0], node.p[4:7, 1], c='blue', s=800, marker='s')
        if node.code !=[]:
            # node.get_logger().info(f'code: {node.code}')
            # node.get_logger().info(f'px: {node.p[:7, 0]}')
            for i in range(node.p.shape[0]):
                p=node.p[i]
                # node.get_logger().info(f'p: {node.p[i]}')
                plt.annotate('RM'+str(i)+':'+re.sub('\D', '', str(node.code))[2*i:2*(i+1)], 
                             xy=(p[0], p[1]), xytext=(p[0], p[1]+0.1))
            plt.scatter(node.tp[1:7, 0], node.tp[1:7, 1], c='m', s=455, marker='d')
            for i in range(1, node.tp.shape[0]):
                p=node.tp[i]
                # node.get_logger().info(f'p: {node.p[i]}')
                plt.annotate('tp of '+str(i), xy=(p[0], p[1]), xytext=(p[0], p[1]+0.1))
        plt.xlim(-3, 3)
        plt.ylim(-2.5,2.5)
        # plt.axes().set_facecolor('black')
        plt.grid()
        plt.pause(0.02)
        fig1.clf()

    node.destroy_node()
    rclpy.shutdown()
