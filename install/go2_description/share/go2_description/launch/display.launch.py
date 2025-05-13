import launch
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os

def generate_launch_description():
    # 获取功能包路径
    go2_description_pkg = get_package_share_directory("go2_description")
    use_joint_state_pulisher = DeclareLaunchArgument(
        name="use_joint_state_pulisher",
        default_value="True",
        description="Flag to enable joint_state_publisher"
    )
    # 使用xacro读取urdf文件中的内容
    robot_desc = ParameterValue(Command(["xacro ",os.path.join(go2_description_pkg, "urdf", "go2_description.urdf")]))
    # robot_state_publisher --- 加载机器人 urdf 文件
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable= "robot_state_publisher",
        parameters=[{"robot_description":robot_desc}]  
    )
    # joint_state_publisher --- 发布关节状态
    # 以后更合理的方式是由程序动态获取关节信息并发布
    # 这个节点的启动应该灵活启动，有条件
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=IfCondition(LaunchConfiguration("use_joint_state_pulisher"))
    )

    return LaunchDescription([
        use_joint_state_pulisher,
        robot_state_publisher,
        joint_state_publisher,
       
    ])

