from ros2_interfaces.msg import Motions
import rclpy
from rclpy.node import Node

class Inbetween(Node):
    def __init__(self,name):
        super().__init__(name)
        self.pose_pub=self.create_publisher(Motions,"motions_b", 10)
        self.pose_sub=self.create_subscription(Motions, 'motions', self.sub_callback, 10)
        self.timer=self.create_timer(0.05, self.timer_callback)
        self.get=False
        self.msg=Motions()


    def timer_callback(self):
        self.pose_pub.publish(self.msg)

    
    def sub_callback(self, msg=None):
        if msg==None:
            self.get_logger().error('msg is none')
        else:
            self.get=True
            self.msg=msg


def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = Inbetween("inbetween")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    node.destroy_node()
    rclpy.shutdown() # 关闭rclpy