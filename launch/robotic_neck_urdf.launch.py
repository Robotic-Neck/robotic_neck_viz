from launch import LaunchDescription
from launch_ros.actions import Node

from launch_utils.utils import launch_rviz_node
from launch_utils.utils import launch_robot_state_publisher_node as launch_rsp_node


def generate_launch_description():
    launchs = []

    launchs.append(Node(package='robotic_neck_viz', executable='neck_joint_publisher', name='neck_joint_publisher'))
    launchs.append(launch_rsp_node( package_name="robotic_neck_viz", xacro_file="robotic_neck.urdf.xacro"))
    launchs.append(launch_rviz_node(package_name='robotic_neck_viz', config_file='robotic_neck.rviz'))
    launchs.append(Node(package='rqt_reconfigure',executable='rqt_reconfigure', name='rqt_reconfigure'))

    return LaunchDescription(launchs)
