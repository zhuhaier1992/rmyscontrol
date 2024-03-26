#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros2_interfaces.msg import Motions, Ctrl
from ros2_interfaces.srv import AgentStatus
from geometry_msgs.msg import Twist, Vector3, Pose2D
import time
from roscpp.utils import *
import numpy as np
from matplotlib import pyplot as plt
import re


no_rm=100
class Draw(Node):
    def __init__(self, name):
        super().__init__(name)
        self.name=name
        self.get_logger().info(f'node started')
        self.mo_sub=self.create_subscription(Motions, 'rvo2', self.update_pv, 10)
        # self.ctrl_sub=self.create_subscription(Ctrl, 'to_rvo', self.calc_v, 10)
        # self.rvo_pub=self.create_publisher(Ctrl, 'rvo', 10)
        self.p=np.zeros([no_rm+1,3])
        self.v=np.zeros([no_rm+1,3])
        self.tp=np.zeros([no_rm+1,3])
        self.code=[]
        

    def update_pv(self, msg):
        if len(msg.poses)!=0:
            self.p=pose2d_to_nparray(msg.poses)
            self.v=pose2d_to_nparray(msg.twists)

    # def calc_v(self, msg):
    #     self.code=msg.code
    #     self.tp=pose2d_to_nparray(msg.pose)

def main(args=None):
    rclpy.init(args=args)
    node = Draw(name='draw')
    fig1=plt.figure('frame', facecolor='black')
    while rclpy.ok():
        rclpy.spin_once(node)
        node.get_logger().info(f'node running')
        plt.scatter(node.p[:, 0], node.p[:, 1], c='red', s=40, marker='o')
        # if node.code !=[]:
        #     # node.get_logger().info(f'code: {node.code}')
        #     # node.get_logger().info(f'px: {node.p[:7, 0]}')
        #     for i in range(node.p.shape[0]):
        #         p=node.p[i]
        #         # node.get_logger().info(f'p: {node.p[i]}')
        #         plt.annotate('RM'+str(i)+':'+re.sub('\D', '', str(node.code))[2*i:2*(i+1)], 
        #                      xy=(p[0], p[1]), xytext=(p[0], p[1]+0.1))
        #     plt.scatter(node.tp[1:7, 0], node.tp[1:7, 1], c='m', s=455, marker='d')
        #     for i in range(1, node.tp.shape[0]):
        #         p=node.tp[i]
        #         # node.get_logger().info(f'p: {node.p[i]}')
        #         plt.annotate('tp of '+str(i), xy=(p[0], p[1]), xytext=(p[0], p[1]+0.1))
        plt.xlim(-100, 100)
        plt.ylim(-100,100)
        # plt.axes().set_facecolor('black')
        plt.grid()
        plt.pause(0.01)
        fig1.clf()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()