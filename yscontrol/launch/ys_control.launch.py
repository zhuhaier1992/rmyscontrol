# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """launch内容描述函数，由ros2 launch 扫描调用"""
    YS1 = Node(
        package="yscontrol",
        executable="yscontrol",
        name='YS1',
        remappings=[
            ('/motion_capture', '/YS1'),
        ]
    )
    YS2 = Node(
        package="yscontrol",
        executable="yscontrol",
        name='YS2',
        remappings=[
            ('/motion_capture', '/YS2'),
        ]
    )
    YS3 = Node(
        package="yscontrol",
        executable="yscontrol",
        name='YS3',
        remappings=[
            ('/motion_capture', '/YS3'),
        ]
    )
    YS4 = Node(
        package="yscontrol",
        executable="yscontrol",
        name='YS4',
        remappings=[
            ('/motion_capture', '/YS4'),
        ]
    )
    YS5 = Node(
        package="yscontrol",
        executable="yscontrol",
        name='YS5',
        remappings=[
            ('/motion_capture', '/YS5'),
        ]
    )
    YS6 = Node(
        package="yscontrol",
        executable="yscontrol",
        name='YS6',
        remappings=[
            ('/motion_capture', '/YS6'),
        ]
    )

    # 创建LaunchDescription对象launch_description,用于描述launch文件 RM4, RM5, RM6
    launch_description = LaunchDescription(
        [ YS1, YS2, YS3, YS4, YS5, YS6])# YS1, YS2, YS3, YS4, YS5, YS6
    # 返回让ROS2根据launch描述执行节点
    return launch_description