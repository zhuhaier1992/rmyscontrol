import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
# from robomaster_msgs.action import Move

import threading
import random
import time
import math
from .mods import YanAPI
import numpy as np
from numpy import pi
from numpy.linalg import norm

from geometry_msgs.msg import Twist, Vector3, Pose2D

# customized Message types
from ros2_interfaces.msg import Motions
from ros2_interfaces.msg import Yscomm
from ros2_interfaces.srv import Comm




################ Yanshee Internal State def ##
BOT_STANDBY = 0
BOT_APPROACHING = 1
BOT_BALL_READY = 2
ip_addr_1= "192.168.3.51"  #6AB8
ip_addr_2= "192.168.3.56"  #A77D
ip_addr_3="192.168.3.76"   #DF7D
ip_addr_4="192.168.3.64"   #9B3D
ip_addr_5="192.168.3.54"   #2DB4
ip_addr_6="192.168.3.53"   #B634
ips=[ip_addr_1, ip_addr_2, ip_addr_3, ip_addr_4, ip_addr_5, ip_addr_6]


################# position message format #####
MSG_POS_BALL_INDEX = 0
anchors=np.array([[-2.5, 0.2, 0],
                  [-2.4, -0.5, 0],
                  [-2.5, 1.0, 0],
                  [2.5, 0, pi],
                  [2.5, -0.7, pi],
                  [2.5, 1.0, pi]])
NUM_ROBOMASTER = 6
NUM_YANSHEE = 6
PREPARE_DIST=2
MSG_POS_LEN = NUM_ROBOMASTER + NUM_YANSHEE + 1 # 13
MSG_POS_ROBOMASTERS = [x+1 for x in range(NUM_ROBOMASTER)] # [1,2,3,4,5,6]
MSG_POS_YANSHEES = [y+1+NUM_ROBOMASTER for y in range(NUM_YANSHEE)] #[7,8,9,10,11,12]

################# Util functions ###############
def limit_pi(t):
    if t>pi:
        return t-2*pi
    elif t<-pi:
        return t+2*pi
    return t


def dist(x1,y1,x2,y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)

def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) 


def sgn(x):
    if x> 0 :return 1
    elif x<0: return -1
    else: return 0

def pose2d_to_nparray(poses):
    # print(poses)
    if isinstance(poses, Pose2D):
        return np.array([poses.x, poses.y, poses.theta])
    l=len(poses)
    r=np.zeros([l, 3])
    for i in range(l):
        r[i,:]=[poses[i].x, poses[i].y, poses[i].theta]
    return r

################# Yanshee Class ##################

class YsControl(Node):
    cls_total = 0
    def __init__(self, name):
        super().__init__(name)
        self.id=int(self.get_name()[2])
        self.get_logger().info(f'ys started, id: {self.id}')
        self.ip = ips[self.id-1]
        self.anchor = anchors[self.id-1]
        self.limits=(0.2,0.2)
        self.pose = Pose2D()
        self.pb=Pose2D()
        self.vb=Pose2D()
        self.distance_to_ball=10
        self.receive = BOT_APPROACHING

        ###################### communicate with ROS ######################
        # retrieve global pos data as a subscription
        self.retrieve_pos=self.create_subscription(Motions, 'motions', self.get_pos, 10)
        # SERVER
        self.server=self.create_service(Comm,f'/YS{self.id}/ys_comm',self.yscomm_cb) # 'ys_comm' as the server, replies robomaster's requests
        self.pose = 0
        # CLIENT
        self.client = self.create_client(Comm,'rm_comm') # 'rm_comm' as the client, send requests to 'rm_comm' via rclpy.spin_once(self.tick())
        self.req = Comm.Request() # a message class, as an instance of class 'Comm'
        self.req.comm = 0 # state the 'Comm' class instance's attribute '.comm' to 0, stands for 'it has stopped movement and is requiring robomaster to move'
        YanAPI.yan_api_init(self.ip)

    # position
    def get_pos(self,msg=[]): # subscription of publisher 'motions_b'
        '''
        subscription: 'motions_b', update and return self.pose in Pose2D format
        '''
        if msg == []:
            self.get_logger().error('empty message [] recieved')
        else:
            try:
                self.pose = msg.poses[self.id+6]
                self.pb=msg.poses[0]
                self.vb=msg.twists[0]
                self.distance_to_ball=distance(pose2d_to_nparray(self.pb), pose2d_to_nparray(self.pose))
            except IndexError:
                self.get_logger().warn(f'warn: msg.poses{self.id}: len p: {len(msg.poses)}')
        

    def move_to_specific_direction(self, d):
        # self.get_logger().info(f'{self.pose.theta}')
        r_d= limit_pi(d - self.pose.theta)
        if self.kickable():
            if self.id!=6: # 6号会摔
                self.kick()
        elif self.distance_to_ball>PREPARE_DIST:
            if abs(r_d)>0.15:
                self.get_logger().info(f'desired: {d}, self: {self.pose.theta}')
            if r_d>0.35:
                YanAPI.sync_play_motion('OneStepTurnLeft', speed='very fast', repeat=1)
                self.get_logger().info(f'turning left, dtheta: {r_d}')
            elif r_d>0.15:
                YanAPI.sync_play_motion('TurnL_tiny', speed='very fast', repeat=1)
                self.get_logger().info(f'turning left tiny, dtheta: {r_d}')
            elif r_d<-0.35:
                YanAPI.sync_play_motion('OneStepTurnRight', speed='very fast', repeat=1)
                self.get_logger().info(f'turning right tiny, dtheta: {r_d}')
            elif r_d<-0.15:
                YanAPI.sync_play_motion('TurnR_tiny', speed='very fast', repeat=1)
                self.get_logger().info(f'turning right tiny, dtheta: {r_d}')
            YanAPI.stop_play_motion()
            time.sleep(1)
        rclpy.spin_once(self)

    def kickable(self):
        pb=pose2d_to_nparray(self.pb)
        vb=pose2d_to_nparray(self.vb)
        p=pose2d_to_nparray(self.pose)
        if distance(pb, p)-5*norm(vb[:2])<0.4:
            return True
        else:
            return False
        

    def move_to_position(self, x, y):
        if self.id==6:#6号异动，无法理解
            return
        x_d=x-self.pose.x
        y_d=y-self.pose.y
        if self.kickable():
            self.kick()
        elif self.distance_to_ball>PREPARE_DIST:
            if abs(x_d)<=0.05 and abs(y_d)<=0.05: # already in anchor
                # if self.id not in [1, 2,5] and random.random()>0.5:
                #     motion_list=['left', 'right']
                #     random_motion=motion_list[round(random.random())]
                #     YanAPI.sync_play_motion('walk', random_motion, speed='fast', repeat=1)
                # else:
                #     time.sleep(1)
                time.sleep(0.1)
            elif self.anchor[2]==0:
                if abs(x_d)>0.05:
                    self.get_logger().info(f'x d: {x_d}')
                if x_d<-0.05:
                    YanAPI.sync_play_motion('walk', 'backward', speed='fast', repeat=1)
                elif x_d>0.05:
                    YanAPI.sync_play_motion('walk', 'forward', speed='fast', repeat=1)
                
                if abs(y_d)>0.05:
                    self.get_logger().info(f'y d: {y_d}')
                if y_d<-0.05:
                    YanAPI.sync_play_motion('walk', 'right', speed='fast', repeat=1)
                elif y_d>0.05:
                    YanAPI.sync_play_motion('walk', 'left', speed='fast', repeat=1)
            else:
                if abs(x_d)>0.05:
                    self.get_logger().info(f'right side: x d: {x_d}')
                if x_d<-0.05:
                    YanAPI.sync_play_motion('walk', 'forward', speed='fast', repeat=1)
                elif x_d>0.05:
                    YanAPI.sync_play_motion('walk', 'backward', speed='fast', repeat=1)
                
                if abs(y_d)>0.05:
                    self.get_logger().info(f'y d: {y_d}')
                if y_d<-0.05:
                    YanAPI.sync_play_motion('walk', 'left', speed='fast', repeat=1)
                elif y_d>0.05:
                    YanAPI.sync_play_motion('walk', 'right', speed='fast', repeat=1)
            
        YanAPI.stop_play_motion()
        time.sleep(0.5)
        rclpy.spin_once(self)


    def yscomm_cb(self, request, response): 
        self.get_logger().info(f'get {request.comm} from rm')
        self.receive = request.comm
        response.res=1
        return response


    def send_kick(self, comm):
        self.req.comm = comm
        self.client.call_async(self.req)


    def kick(self): # pure kick
        YanAPI.yan_api_init(self.ip)
        YanAPI.sync_play_motion(name='Football_RShoot',speed='normal',repeat=1)
        YanAPI.stop_play_motion()
        # time.sleep(1)
    
    def wave(self):
        pb=pose2d_to_nparray(self.pb)
        p=pose2d_to_nparray(self.pose)
        if distance(pb, p)<1.3:
            YanAPI.sync_play_motion(name='GoalKeeper1')
            YanAPI.stop_play_motion()
    
    def stop():
        YanAPI.stop_play_motion()
    
    
def main(args=None):
    rclpy.init(args=args)
    bot = YsControl(name='YS2')
    while rclpy.ok():
        rclpy.spin_once(bot) # update pos
        if bot.receive == BOT_BALL_READY: #2
            bot.kick() # very short
            bot.send_kick(1)
            bot.receive=1
        elif bot.receive == BOT_APPROACHING: #1
            bot.move_to_specific_direction(bot.anchor[2])
            bot.move_to_position(bot.anchor[0], bot.anchor[1])
            bot.wave()
        elif bot.receive==BOT_STANDBY: #0
            bot.stop()
            time.sleep(0.1)
        else:
            bot.get_logger().warn('main loop: bot.receive is not any of 0,1,2')
    bot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
