from launch import LaunchDescription
from launch_ros.actions import Node

from launch_utils.utils import launch_rviz_node
from launch_utils.utils import launch_robot_state_publisher_node as launch_rsp_node
from launch_utils.utils import launch_joint_state_publisher_node as launch_jsp_node


def generate_launch_description():
    launchs = []
    
    launchs.append(launch_rsp_node(package_name="robotic_neck_viz", xacro_file="robot.urdf.xacro"))
    launchs.append(launch_jsp_node(gui=True))
    launchs.append(launch_rviz_node(package_name='robotic_neck_viz', config_file='wamv.rviz'))
    launchs.append(Node(package='robotic_neck_viz',executable='laser_sim', name='laser_sim'))
    
    return LaunchDescription(launchs)
