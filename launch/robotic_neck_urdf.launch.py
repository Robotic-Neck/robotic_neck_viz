from launch import LaunchDescription
from launch_ros.actions import Node

from launch_utils.utils import launch_rviz_node, include_launch
from launch_utils.utils import launch_robot_state_publisher_node as launch_rsp_node


def generate_launch_description():
    launchs = []

    # launch robot state model getting from IMU data    
    launchs.append(Node(package='robotic_neck_viz', executable='neck_state_joint_publisher', name='neck_state_joint_publisher'))
    launchs.append(launch_rsp_node( package_name="robotic_neck_viz", xacro_file="robotic_neck_state.urdf.xacro", namespace="state"))
   
    # launch robot target model to apply control by inverse kinematics.
    launchs.append(Node(package='robotic_neck_viz', executable='neck_target_joint_publisher', name='neck_target_joint_publisher'))
    launchs.append(launch_rsp_node( package_name="robotic_neck_viz", xacro_file="robotic_neck_target.urdf.xacro", namespace="target"))
    
    # launch Rviz and RQT Reconfigure
    launchs.append(launch_rviz_node(package_name='robotic_neck_viz', config_file='robotic_neck.rviz'))
    launchs.append(Node(package='rqt_reconfigure',executable='rqt_reconfigure', name='rqt_reconfigure'))

    return LaunchDescription(launchs)
