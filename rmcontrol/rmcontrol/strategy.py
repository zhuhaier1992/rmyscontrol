import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from robomaster_msgs.action import Move
from ros2_interfaces.msg import Motions, Ctrl, Yscomm
from ros2_interfaces.srv import AgentStatus, Comm
# from geometry_msgs.msg import Twist, Vector3, Pose2D
import time
# import message_filters
from .utils import *
import copy
from math import *

#tp为初始目标位置，依次为: 足球、队伍a的3个站位、b的3个站位
tp=np.array([[-0.7       ,  0.        ,  0.        ],
            [-1.5       , -1.2        ,  0.        ],
            [-1.4       ,  0.        ,  0.        ],
            [-1.5       ,  1.2        ,  0.        ],
            [ 1.5       , -1.2        ,  pi],
            [ 1.4       ,  0.        ,  pi],
            [ 1.5       ,  1.2        ,  pi]])
whole_range=[[-3.07,2.7],[-2.26,2.4]]
safe_range_x=[-2.1, 2.0]
safe_range_y=[-2.0, 2.1]
attacker=0
shooter=0
target_poses=tp.copy()
team_a=[1,3,5] # 队伍a由1 3 5号RM组成
team_b=[4,2,6]
team=[team_a, team_b]
compete_range=[-0.8,0.8] # 球的x位置超出范围则切换team
switch_range=[-0.8, 0.8] # 球的y位置超出范围则切换同team的不同kicker
left_players=[team_a[0], team_b[0]] # 左路球员
mid_players=[team_a[1], team_b[1]] # 中路
right_players=[team_a[2], team_b[2]] # 右路
# target_poses类似于tp，只是1~6的排序不是按照队伍，而是RM的编号
for i in range(len(team_a)):
    target_poses[team_a[i]]=tp[i+1]
for i in range(len(team_b)):
    target_poses[team_b[i]]=tp[i+len(team_a)+1]

p_goal_a=np.array([-3.0, 0, 0]) # 上方队伍的球门
p_goal_b=np.array([2.7, 0, 0]) # 下方的球门
p_goal=[p_goal_a, p_goal_b]
atk_comp=1.0
def_dist=1.4
no_rms=7 # 6 rm plus 1 ball
no_ys=6
target_poses=target_poses
vkick_min=0.5 # the least speed for kicker to keep agile
vkikcer_adjust=3 # to adjust ball speed to find kicker
vshoot_adjust=3 # to adjust ball speed to calc shooting position for kicker(to quickly move to stop ball)
vmove_adjust=3 # to add this value times ball speed to adjust final speed of kicker


class Strategy(Node):
    def __init__(self, name):
        super().__init__(name)
        self.name=name
        self.p=np.zeros([no_rms,3])
        self.p_ys=np.zeros([no_ys, 3])
        self.v=np.zeros([no_rms,3])

        self.game_code='reset'   # stop, reset, play
        self.team_code=[0, 0]    # for team [a, b], 0: defense, 1: attack
        self.target_pose=target_poses.copy()
        self.target_code=[0]*no_rms
        self.agent_status=[0]*no_rms # received rms' status: floats
        self.catcher=0
        self.kicker=0
        self.rvo_res=[]
        self.rel_cmd=np.zeros([no_rms,3])
        self.first_reset=1
        self.last_warn_time=time.time()
        self.mo_sub=self.create_subscription(Motions, 'motions', self.mo_cb, 1)
        self.rvo_sub=self.create_subscription(Ctrl, 'rvo2', self.rvo_cb, 1) # change this topic to "rvo2" to receive cpp rvo2 from "roscpp"
        self.status_server=self.create_service(AgentStatus, 'agent_status', self.status_cb)
        self.ctrl_pub=self.create_publisher(Ctrl, 'ctrl', 1)
        self.comm_srv=self.create_service(Comm, 'rm_comm', self.comm_srv_cb)
        self.ys_client=self.create_client(Comm, 'ys_comm')
        self.ys_req=Comm.Request()
        self.kickoff=2  
        self.obtime=0
        # self.comm_pub=self.create_publisher(Yscomm, 'rm_comm', 1)
        # self.comm_sub=self.create_subscription(Yscomm, 'ys_comm', self.comm_cb, 1)
        self.ys_state=0
        self.to_rvo_pub=self.create_publisher(Ctrl, 'to_rvo', 1)
        self.ctrl=Ctrl()
        self.wait_for_ys=False
        self.change_time=time.time()


    def comm_srv_cb(self, req, resp):
        self.get_logger().info(f'rm_comm got: {req.comm}')
        self.ys_state=req.comm
        if req.comm==0:
            self.wait_for_ys=True
        elif req.comm>=1:
            self.wait_for_ys=False

        resp.res=1
        return resp
        

    def rvo_cb(self, msg):
        self.rvo_res=pose2d_to_nparray(msg.pose)

    def status_cb(self, req, resp):
        if req.id==self.catcher and req.status==30:
            self.get_logger().info(f'catcher RM{req.id} status: {req.status}')
        # self.get_logger().info(f'catcher is {self.catcher}: {self.agent_status[self.catcher]}')
        self.agent_status[req.id]=req.status
        resp.res=1
        return resp

    def mo_cb(self, mc_msg):
        if len(mc_msg.poses)==no_rms+no_ys:
            self.p=pose2d_to_nparray(mc_msg.poses[:no_rms])
            self.p_ys=pose2d_to_nparray(mc_msg.poses[no_rms:no_rms+no_ys])
        else:
            t=time.time()
            if t-self.last_warn_time>2:
                self.get_logger().warn(f'mc p len: {len(mc_msg.poses)}')
                self.last_warn_time=t
        if len(mc_msg.twists)==no_rms+no_ys:
            self.v=pose2d_to_nparray(mc_msg.twists[:no_rms])

    def force_kicker(self):
        '''
        当球的x坐标超出范围后，强制某队变成进攻队伍，从而增加来回往复的次数，以免直接出界。
        '''
        def find_y(y, team):
            '''
            根据当前球的y坐标（横向位置）和球队，决定kicker
            '''
            if team=='a':
                if y>switch_range[1]:
                    self.kicker=team_a[2]
                elif y<switch_range[0]: self.kicker=team_a[0]
                else: self.kicker=team_a[1]
            else: 
                if y>switch_range[1]:
                    self.kicker=team_b[2]
                elif y<switch_range[0]: self.kicker=team_b[0]
                else: self.kicker=team_b[1]
        bx=self.p[0][0]
        by=self.p[0][1]
        if bx>0:
            find_y(by, 'b')
        else:
            find_y(by, 'a')

    def switch_kicker(self):
        '''
        有时根据find_closest找到的kicker并不合适，可能导致球员越界太多（如左路的跑到右路去了）
        因此若球的y坐标越界则强制切换球员。
        '''
        atk=0
        if self.kicker in team_a:
            atk=team_a
        else:
            atk=team_b
        bp=self.p[0]
        by=bp[1]
        if self.kicker not in left_players and by<switch_range[0]:
            self.kicker=atk[0]
        elif self.kicker not in right_players and by>switch_range[1]:
            self.kicker=atk[2]
        elif self.kicker not in mid_players and abs(by)<0.8:
            self.kicker=atk[1]

    def out_of_boundary(self):
        pb=self.p[0]
        if pb[1]<safe_range_y[0]:
            return 1
        elif pb[1]>safe_range_y[1]:
            return 3
        elif pb[0]>safe_range_x[1] or pb[0]<safe_range_x[0]:
            return 2
        else:
            return 0
        
    def out_side(self):
        pb=self.p[0]
        if pb[1]<safe_range_y[0] or pb[1]>safe_range_y[1]:
            return True
        else:
            return False
        
    def out_goal(self):
        pb=self.p[0]
        if pb[0]<safe_range_x[0] or pb[0]>safe_range_x[1]:
            return True
        else:
            return False
        
    def get_catcher(self):
        pb=self.p[0]
        # if pb[1]<target_poses[1][1]:
        #     self.catcher=1
        # elif pb[1]>target_poses[3][1]:
        #     self.catcher=3
        # else:
        #     self.catcher=2
        if pb[0]<0:
            self.catcher=3
            self.kickoff=2
        elif pb[0]>=0:
            self.catcher=2
            self.kickoff=5

    def command(self):
        self.ctrl=Ctrl(code=self.target_code, pose=nparray_to_pose2d(self.rel_cmd))
        self.ctrl_pub.publish(self.ctrl)
        
    def team_alloc(self):
        if self.kicker in team_a:
            self.team_code=[1,0]
        elif self.kicker in team_b:
            self.team_code=[0,1]
        else:
            self.team_code=[0,0]

    def tp_alloc(self):
        '''
        分配每队的target pose
        '''
        for i in team_a:
            self.target_pose[i]=np.array([self.p[0][0] - def_dist + \
                self.team_code[0]*atk_comp, target_poses[i][1], 
                calc_aiming_goal(self.p[i], p_goal[1])])
        for i in team_b:
            self.target_pose[i]=np.array([self.p[0][0] + def_dist - \
                self.team_code[1]*atk_comp, target_poses[i][1], 
                calc_aiming_goal(self.p[i], p_goal[0])])
            
    def reset(self):
        if self.first_reset:
            self.get_catcher()
            self.kicker=0
            self.target_code=[10]*7
            self.target_code[self.catcher]=30
            self.target_pose=target_poses.copy()
            self.first_reset=0
            self.call_yscomm('all', 0)
            self.get_logger().info(f'call ys to stop. RM{self.catcher} starts to catch')
        self.catch_ball(True)


    def call_yscomm(self, id, c):
        if id=='all':
            for i in range(1, 7):
                self.ys_client=self.create_client(Comm, f'/YS{i}/ys_comm')
                self.ys_req=Comm.Request()  
                self.ys_req.comm=c
                self.ys_client.call_async(self.ys_req)
        else:
            self.ys_client=self.create_client(Comm, f'/YS{id}/ys_comm')
            self.ys_req=Comm.Request()  
            self.ys_req.comm=c
            self.ys_client.call_async(self.ys_req)


    def catch_ball(self, to_ys=False):
        if self.target_code[self.catcher]<=30:  # move to ball
            rclpy.spin_once(self)
            self.catch_pose=calc_catching_pose(self.p[0], self.p[self.catcher], self.v[0])
            self.target_pose[self.catcher]=self.catch_pose
            # self.get_logger().info(f'catcher state: {self.agent_status[self.catcher]}')
            # self.get_logger().info(f'dist: {distance(self.p[self.catcher], self.target_pose[self.catcher])}')
            if self.agent_status[self.catcher]==30 and check_catchable(self.p[self.catcher], self.p[0]):
                self.target_code[self.catcher]=32
                self.get_logger().info(f'RM{self.catcher} ready to grip, code: {self.target_code[self.catcher]}')
        elif self.target_code[self.catcher]==32: # grip
            # self.get_logger().info(f'agent status: {self.agent_status[self.catcher]}')
            # dphi, d=check_catched(self.p[self.catcher], self.p[0], self.v[0])
            # self.get_logger().info(f'dphi: {dphi}, d: {d}')
            # self.get_logger().info(f'catcher state: {self.agent_status[self.catcher]}')
            if self.agent_status[self.catcher]==32: # finished grip
                d, dphi, ccres=check_catched(self.p[self.catcher], self.p[0], self.v[0])
                self.get_logger().info(f'dist to ball: {d}, dphi: {dphi}')
                if ccres:
                    self.get_logger().info(f'---RM{self.catcher} caught ball')
                    self.target_code[self.catcher]=34
                else: # recatch
                    self.get_logger().info(f'recatch')
                    self.target_code[self.catcher]=30
        elif self.target_code[self.catcher]==34: # if got ball, then take it to the yanshee kicker
            if to_ys:
                if self.kickoff==2:
                    self.target_pose[self.catcher]=np.array([self.p_ys[1][0]+0.41, self.p_ys[1][1]-0.03, pi])
                elif self.kickoff==5:
                    self.target_pose[self.catcher]=np.array([self.p_ys[4][0]-0.41, self.p_ys[4][1]+0.04, 0])
            else:
                self.target_pose[self.catcher]=target_poses[0].copy()
            # self.get_logger().info(f'check if RM{self.catcher} catched {distance(self.p[self.catcher], self.target_pose[self.catcher])}')
            # self.get_logger().info(f'{self.p[self.catcher]}, {self.target_pose[self.catcher]}，{self.p_ys[1]}')
            if in_position(self.p[self.catcher], self.target_pose[self.catcher], 0.04):
                self.get_logger().info(f'---RM{self.catcher} carried ball to position')
                self.target_code[self.catcher]=35
        elif self.target_code[self.catcher]==35: # ready to release ball
            self.get_logger().info(f'catcher status:{self.agent_status[self.catcher]},code:{self.target_code[self.catcher]}')
            # if self.agent_status[self.catcher]==35:
            #     self.get_logger().info(f'---RM{self.catcher} released ball')
            self.target_code[self.catcher]=36
        
        elif self.target_code[self.catcher]==36: # ball in position
            self.target_pose[self.catcher]=target_poses[self.catcher].copy()
            # 放下球后先别转身，否则容易撞到球。
            if target_poses[self.catcher][2]==0:
                self.target_pose[self.catcher][2]=pi
            elif target_poses[self.catcher][2]>3:# pi
                self.target_pose[self.catcher][2]=0
            # self.get_logger().info(f'catcher: {self.p[self.catcher]}, target: {target_poses[self.catcher]}')
            if in_position(self.p[self.catcher], self.target_pose[self.catcher]):
                self.target_code[self.catcher]=37
        elif self.target_code[self.catcher]==37: # rm in position but has to rotate
            self.target_pose[self.catcher]=target_poses[self.catcher].copy()
            if in_position(self.p[self.catcher], self.target_pose[self.catcher]):
                self.get_logger().info(f'---all in position')
                self.get_logger().info(f'{self.catcher} code: {self.target_code[self.catcher]}')
                self.target_code[self.catcher]=39
        elif self.target_code[self.catcher]==39: # all in position
            self.wait_for_ys=True
            self.get_logger().info(f'reset complete, call ys to kick')
            self.catcher=0
            self.call_yscomm(self.kickoff, 2)
            self.ys_state=0
            while self.ys_state==0:
                rclpy.spin_once(self)
                time.sleep(0.1)
            self.call_yscomm('all', 1)
            # time.sleep(1)
            self.game_code='play' # play
            self.target_code=[10]*no_rms
            self.first_reset=1
            self.obtime=0
    

def main(args=None):
    global target_poses
    rclpy.init(args=args)
    node = Strategy(name='strategy')
    while rclpy.ok():
        rclpy.spin_once(node)
        if len(node.p)==0:
            continue

        if node.game_code=='reset':
            node.reset()
        else:
            oob=node.out_of_boundary()
            if oob: 
                node.obtime+=1
                if node.obtime%200==0:
                    node.get_logger().info(f'oob count: {node.obtime}')

                if node.obtime>1000:
                    node.obtime=0
                    node.get_logger().info(f'ball out of boundary, reset')
                    node.game_code='reset'
            else:
                node.obtime=0
                # out_side=node.out_side()
                # out_goal=node.out_goal()
                # if out_side: # 两边出界
                #     node.game_code='reset'
                #     node.get_logger().info(f'ball out of side, reset')
                # elif out_goal: # 靠近球门处出界
                #     node.wait_for_ys=False
                #     # rclpy.spin_once(node)
                #     if node.wait_for_ys:
                #         node.target_code=[0]*7
                #         node.command()
                #         # time.sleep(0.3)
                #     else:
                #     oob2=node.out_of_boundary()
                #     if oob2:
                #         # node.get_logger().info(f'ball in yanshee area and aint come back, reset')
                #         node.game_code='reset'
                #     else:
                #         node.get_logger().info(f'yanshee kicked ball back to play area')
                #         node.game_code='play'
            
        if node.game_code=='play':
            '''
            The ball usually goes unblocked to the border, which look bad.
            To avoid this consequence, when ball is away from the center area
            (out of compete range), force the defence team to have the kicker.
            '''
            if node.p[0][0]>compete_range[1] or node.p[0][0]<compete_range[0]:
                node.force_kicker()
            else:
                kicker,_=find_closest(node.p, vkikcer_adjust*node.v[0]) # ball velocity adjust
                node.kicker=kicker
                node.switch_kicker()
            
            # 强制切换左中右路
            

            node.target_code=[10]*no_rms
            node.team_alloc()
            node.tp_alloc()
            node.target_pose[node.kicker]=goal_position(node.p[0], p_goal[node.team_code.index(0)])+vshoot_adjust*np.array([node.v[0][0],node.v[0][1], 0])
            if in_position(node.p[node.kicker], node.target_pose[node.kicker]+np.array([node.v[0][0],node.v[0][1], 0]), 0.11, 0.16): # 踢球不一定要踢准
                # node.get_logger().info(f'kicker {node.kicker} kick...')
                node.target_code[node.kicker]=20

        if node.catcher!=0: # 先复制一个，以区分对其他tp的clip
            tp_catch=node.target_pose[node.catcher].copy()
        np.clip(node.target_pose, [safe_range_x[0], safe_range_y[0],-4],
            [safe_range_x[1], safe_range_y[1],4], out=node.target_pose)
        if node.catcher!=0:
            if node.target_code[node.catcher]<33: # 追球阶段，对目标位置加上球的速度，加速追上
                node.target_pose[node.catcher]=node.catch_pose+np.array([1.3*node.v[0][0], 1.5*node.v[0][1], 0])
            else:
                node.target_pose[node.catcher]=tp_catch

        node.to_rvo_pub.publish(Ctrl(code=node.target_code, pose=nparray_to_pose2d(node.target_pose)))
        rclpy.spin_once(node)

        i=0
        for p, tp in zip(node.p, node.target_pose):
            if i!=0 and i==node.kicker: # 对kicker速度处理
                node.rel_cmd[i]=trans_relative_co(p, tp + vmove_adjust*node.v[0])
                vn=np.array([node.rel_cmd[i][0], node.rel_cmd[i][1]])
                if norm(vn)<vkick_min: # kicker不能太慢，容易漏球
                    vnew=vn/norm(vn)*vkick_min
                    node.rel_cmd[i]=np.array([vnew[0], vnew[1], node.rel_cmd[i][2]])
            elif len(node.rvo_res)==0 or distance(p, tp)<0.06: # 距离目标位置很近时不用rvo
                node.rel_cmd[i]=trans_relative_co(p, tp)
                if i==node.catcher:
                    node.rel_cmd[i]=np.array([tanh(node.rel_cmd[i][0]),node.rel_cmd[i][1],pi/1.5*tanh(node.rel_cmd[i][2]/pi*4)])
            else:
                node.rel_cmd[i]=node.rvo_res[i]
                # if abs(node.rel_cmd[i][2])>pi/8:
                #     node.rel_cmd[i]=np.array([0,0,node.rvo_res[i][2]])
                # elif i==node.catcher:
                #     # node.rel_cmd[i]=np.array([node.rel_cmd[i][0],node.rel_cmd[i][1], node.rel_cmd[i][2]])
                #     node.rel_cmd[i]=too_slow(node.rel_cmd[i])
            i+=1
        node.command()
    node.destroy_node()
    rclpy.shutdown()


def too_slow(v):
    r=v.copy()
    if v[0]==0:
        v[0]=0.003
    if v[1]==0:
        v[1]=0.003
    if abs(v[0])<0.002:
        r[0]=v[0]/abs(v[0])*0.002
    if abs(v[1])<0.04:
        r[1]=v[1]/abs(v[1])*0.04
    return r