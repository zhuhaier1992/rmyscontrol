import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from robomaster_msgs.action import MoveArm, GripperControl
from robomaster_msgs.msg import GripperState
from ros2_interfaces.msg import Ctrl, State
from std_msgs.msg import ColorRGBA
from ros2_interfaces.srv import AgentStatus
from geometry_msgs.msg import Twist, Vector3, Pose2D, Point, PointStamped
import time
from rclpy.parameter import Parameter

from .utils import *

# arm_pose: high for move, low for catch
arm_pose=[MoveArm.Goal(x=0.15, z=0.06), MoveArm.Goal(x=0.18, z=-0.07)] # up and down position
gripper=[GripperControl.Goal(target_state=1, power=1.), GripperControl.Goal(target_state=2, power=1.)] # open and close
attacker=0
team_a=[1,3,5]
team_b=[4,2,6]
kick_speed=1.5
# action_gap=0.05

class AgentControl(Node):
    def __init__(self, name):
        super().__init__(name)
        self.name=name
        self.id=int(self.get_name()[2])
        self.get_logger().info(f"节点已启动：{self.get_name()}!")

        self.goal_pub=self.create_publisher(Twist, f'/RM{self.id}/cmd_vel',1)
        self.state_pub=self.create_publisher(State, f'/RM{self.id}/state_id', 2)
        self.arm_sub=self.create_subscription(PointStamped, f'/RM{self.id}/arm_position', self.update_arm, 1)
        self.grip_sub=self.create_subscription(GripperState, f'/RM{self.id}/gripper_state', self.update_grip, 1)
        self.arm_action=ActionClient(self, MoveArm, f'/RM{self.id}/move_arm')
        self.gripper_action=ActionClient(self, GripperControl, f'/RM{self.id}/gripper')
        self.arm_goal=arm_pose[0]
        self.gripper_goal=gripper[0]
        self.grip_state=0
        self.grip_start=0
        self.gripper_action.send_goal_async(gripper[0]) # open gripper
        self.release_start=0
        # time.sleep(10)
                
        self.arm_done=0
        self.arm_action.send_goal_async(arm_pose[0]) # init arm pose
        # fut.add_done_callback(self.arm_done_cb)
        self.arm_pos=Point()
        self.arm_start=0

        self.create_subscription(Ctrl, 'ctrl', self.ctrl, 10)
        self.strategy_cli = self.create_client(AgentStatus, 'agent_status')
        self.status=10 
        self.code=0 # 0: stop 1: move 2: kick 3: catch

        self.target=Pose2D()
        self.req_s=AgentStatus.Request()
        self.req_s.id=self.id
        self.req_s.status=self.status

        # set LED color
        if self.id in team_a:
            self.color=ColorRGBA(r=80., g=30., b=30., a=0.5)
        else:
            self.color=ColorRGBA(r=10., g=0., b=200., a=0.5)
        self.create_publisher(ColorRGBA, f'/RM{self.id}/leds/color',5).publish(self.color)
        rclpy.spin_once(self)

    def ctrl(self, msg):
        self.code=msg.code[self.id]
        self.target=msg.pose[self.id]

    def stop(self):
        # self.status=0
        self.goal_pub.publish(Twist(linear=Vector3(x=0., y=0.), angular=Vector3(z=0.)))

    def kick(self):
        global kick_speed
        if self.id in [4,5,6]:
            kick_speed=0.8
        else:
            kick_speed=1.5 
        self.status=25
        self.sync_status()
        self.get_logger().info(f'RM{self.id} starts to kick')
        # kick
        self.goal_pub.publish(Twist(linear=Vector3(x=kick_speed, y=0.), angular=Vector3(z=0.)))
        time.sleep(0.5)
        # self.stop()
        # get back
        self.goal_pub.publish(Twist(linear=Vector3(x=-kick_speed, y=0.), angular=Vector3(z=0.)))
        time.sleep(0.5)
        self.status=10
        self.sync_status()
        self.get_logger().info(f'RM{self.id} finishes kick')
        self.stop()

    def arm_done_cb(self, future):
        goal_handle=future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        """获取结果反馈"""
        result = future.result().result
        self.get_logger().info(f'Result: {result}')



    def gripper_fbcb(self, msg):# msg.feedback.current_state: 1: open 2: closed 0: pause
        cs=msg.feedback.current_state
        # self.get_logger().info(f'current state: {cs}')
        self.grip_state=cs


    def arm_fbcb(self, msg):
        self.get_logger().info(f'arm state: {msg.feedback}')
        # if msg.feedback.progress==1:
        #     self.arm_done=1


    def update_arm(self, msg):
        # if self.id==1:
        #     self.get_logger().info(f'arm state: x {msg.point.x} z {msg.point.z}')
        self.arm_pos=msg.point

    def update_grip(self, msg):
        self.get_logger().info(f'grip state: {msg}')
        # self.grip_state=msg.state


    def check_arm(self):
        # self.get_logger().info(f'arm x: {self.arm_pos.x}')
        if self.arm_pos.x <0.155:
            return 1 # up
        elif self.arm_pos.x >0.175:
            return 2 # down
        
    def sync_status(self):
        self.req_s.status=self.status
        self.strategy_cli.call_async(self.req_s)
        self.state_pub.publish(State(state=self.status))


    @staticmethod
    def pose2d_to_twist(pose: Pose2D):
        pose=clip_pose2d(pose, [-0.5,-0.5], [0.5,0.5])
        return Twist(linear=Vector3(x=pose.x, y=pose.y), angular=Vector3(z=pose.theta))



def main(args=None):
    rclpy.init(args=args)
    node = AgentControl(name='RM1')
    time.sleep(1)
    while rclpy.ok():
        rclpy.spin_once(node)
        # node.req_s.status=node.status
        # node.strategy_cli.call_async(node.req_s)
        # node.state_pub.publish(State(state=node.status))
        # node.get_logger().info(f'----code:{node.code}, status: {node.status}')
        if node.code==0:
            node.stop()
        elif node.code==10:
            # node.status=10
            node.goal_pub.publish(node.pose2d_to_twist(node.target))
        elif node.code==20:
            node.status=20
            node.get_logger().info(f'RM{node.id} kickign, status: {node.status}')
            node.kick()
        elif node.code==30:
            # node.get_logger().info(f'----code:{node.code}, status: {node.status}')
            if node.status!=30:
                node.release_start=0
                node.status=29
                node.arm_action.send_goal_async(arm_pose[1])
                rclpy.spin_once(node)
                node.gripper_action.send_goal_async(gripper[0], feedback_callback=node.gripper_fbcb)
                rclpy.spin_once(node)
                while not node.check_arm()==2:
                    rclpy.spin_once(node) # spin to update arm_pose
                    time.sleep(0.2)
                # arm down and grip open
                node.status=30
                node.sync_status()
            else:
                node.goal_pub.publish(node.pose2d_to_twist(node.target))
        elif node.code==32: # ready to grip
            node.get_logger().info(f'----ready to grip: {node.code}')
            node.gripper_action.send_goal_async(gripper[1], feedback_callback=node.gripper_fbcb)
            rclpy.spin_once(node)
            time.sleep(0.2)
            while not node.grip_state==2:
                rclpy.spin_once(node) # spin to update arm_pose
                time.sleep(0.1)
            node.get_logger().info(f'RM{node.id} closed grip')
            node.status=32
            node.sync_status()
            node.code=33
        elif node.code==33: # closed grip
            node.stop()
        elif node.code==34: # got ball
            # node.status=32
            # node.get_logger().info(f'----: {node.code}')
            node.goal_pub.publish(node.pose2d_to_twist(node.target))
        elif node.code==35: # release ball
            # if not node.release_start:
            node.release_start=1
            node.gripper_action.send_goal_async(gripper[0], feedback_callback=node.gripper_fbcb)
            rclpy.spin_once(node)
            time.sleep(0.8)
            if node.grip_state==1: # grip opened
                node.goal_pub.publish(node.pose2d_to_twist(Pose2D(x=-0.5)))
                time.sleep(0.1)
                node.get_logger().info(f'----ball released, code:{node.code}, status: {node.status}')
                node.status=35
                node.sync_status()
                # node.code=36
        elif node.code==36: # move to position
            # node.get_logger().info(f'code=36')
            if node.status!=36:
                node.get_logger().info(f'RM{node.id} last step of carrying')
                node.arm_action.send_goal_async(arm_pose[0])
                rclpy.spin_once(node)
                node.status=36
                node.sync_status()
                time.sleep(0.2)
                
            node.goal_pub.publish(node.pose2d_to_twist(node.target))
        elif node.code==39:
            node.get_logger().info(f'RM{node.id} finished carrying')
            node.status=10
            node.sync_status()
            

        


        
    node.destroy_node()
    rclpy.shutdown()

