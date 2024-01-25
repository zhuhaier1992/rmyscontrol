# 导入库
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, TextSubstitution

def generate_launch_description():
    """launch内容描述函数，由ros2 launch 扫描调用"""
    # parameters_basic1 = Node(
    #     package="example_parameters_rclcpp",
    #     namespace="rm1",
    #     executable="parameters_basic",
    #     parameters=[{'rcl_log_level': 40}]
    # )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    # launch_description = LaunchDescription(
    #     [parameters_basic1, parameters_basic2])
    # 返回让ROS2根据launch描述执行节点
    return LaunchDescription([
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robomaster_ros'),
                    'launch',
                    'ep.launch'
                ])
            ]),
            launch_arguments={
                'name': 'RM1',
                'serial_number': '3JKDH2T00159G8',
            }.items()
        ),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robomaster_ros'),
                    'launch',
                    'ep.launch'
                ])
            ]),
            launch_arguments={
                'name': 'RM2',
                'serial_number': '3JKCJC400302GS',
            }.items()
        ),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robomaster_ros'),
                    'launch',
                    'ep.launch'
                ])
            ]),
            launch_arguments={
                'name': 'RM3',
                'serial_number': '3JKCJC400301ZP',
            }.items()
        ),
        # IncludeLaunchDescription(
        #     XMLLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('robomaster_ros'),
        #             'launch',
        #             'ep.launch'
        #         ])
        #     ]),
        #     launch_arguments={
        #         'name': 'RM4',
        #         'serial_number': '3JKCJC400301UD',
        #     }.items()
        # ),
        # IncludeLaunchDescription(
        #     XMLLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('robomaster_ros'),
        #             'launch',
        #             'ep.launch'
        #         ])
        #     ]),
        #     launch_arguments={
        #         'name': 'RM5',
        #         'serial_number': '3JKCJC400301W0',
        #     }.items()
        # ),
        # IncludeLaunchDescription(
        #     XMLLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('robomaster_ros'),
        #             'launch',
        #             'ep.launch'
        #         ])
        #     ]),
        #     launch_arguments={
        #         'name': 'RM6',
        #         'serial_number': '3JKCJC400300Y9',
        #     }.items()
        # ),
    ])

