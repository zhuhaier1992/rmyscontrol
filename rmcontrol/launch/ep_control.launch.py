# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """launch内容描述函数，由ros2 launch 扫描调用"""
    RM1 = Node(
        package="rmcontrol",
        executable="agent_control",
        # ns="RM1",
        remappings=[
            ('/motion_capture', '/RM1'),
        ]
    )
    RM2 = Node(
        package="rmcontrol",
        executable="agent_control",
        name="RM2",
        remappings=[
            ('/motion_capture', '/RM2'),
        ]
    )
    RM3 = Node(
        package="rmcontrol",
        executable="agent_control",
        name="RM3",
        remappings=[
            ('/motion_capture', '/RM3'),
        ]
    )
    RM4 = Node(
        package="rmcontrol",
        executable="agent_control",
        name="RM4",
        remappings=[
            ('/motion_capture', '/RM4'),
        ]
    )
    RM5 = Node(
        package="rmcontrol",
        executable="agent_control",
        name="RM5",
        remappings=[
            ('/motion_capture', '/RM5'),
        ]
    )
    RM6 = Node(
        package="rmcontrol",
        executable="agent_control",
        name="RM6",
        remappings=[
            ('/motion_capture', '/RM6'),
        ]
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件 RM4, RM5, RM6
    launch_description = LaunchDescription(
        [RM1, RM2, RM3, RM4, RM5, RM6])
    # 返回让ROS2根据launch描述执行节点
    return launch_description