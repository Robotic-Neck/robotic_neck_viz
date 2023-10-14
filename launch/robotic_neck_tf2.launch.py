from launch import LaunchDescription
from launch_ros.actions import Node

from launch_utils.utils import launch_rviz_node
from launch_utils.utils import launch_static_tf_node
from launch_utils.utils import launch_robot_state_publisher_node as launch_rsp_node
from launch_utils.utils import launch_joint_state_publisher_node as launch_jsp_node


def generate_launch_description():
    launchs = []
    
    launchs.append(launch_static_tf_node("world", "right_base", translation=[-0.075, -0.05, 0]))
    launchs.append(launch_static_tf_node("right_base", "right_down_tensor", translation=[0.0, 0.0, 0.013]))
    
    launchs.append(launch_static_tf_node("world", "left_base", translation=[-0.075, 0.05, 0]))
    launchs.append(launch_static_tf_node("left_base", "left_down_tensor", translation=[0.0, 0.0, 0.013]))
    
    launchs.append(launch_static_tf_node("world", "base_link", translation=[0, 0, 0.152]))
    
    launchs.append(launch_static_tf_node('universal_joint','neck_link', translation=[0.0,0.0,0.0115]))
    
    launchs.append(launch_static_tf_node('neck_link','right_holder', translation=[-0.075,-0.05,0.0]))
    launchs.append(launch_static_tf_node('right_holder','right_up_tensor', translation=[0.0,0.0,-0.013]))
    
    launchs.append(launch_static_tf_node('neck_link','left_holder', translation=[-0.075,0.05,0.0]))
    launchs.append(launch_static_tf_node('left_holder','left_up_tensor', translation=[0.0,0.0,-0.013]))

    launchs.append(Node(package='robotic_neck_viz', executable='neck_mec_sim', name='robotic_neck_viz'))
    launchs.append(Node(package='robotic_neck_viz', executable='inverse_kinematics', name='inverse_kinematics'))

    launchs.append(launch_rviz_node(package_name='robotic_neck_viz', config_file='robotic_neck.rviz'))
    
    launchs.append(Node(package='rqt_reconfigure',executable='rqt_reconfigure', name='rqt_reconfigure'))

    return LaunchDescription(launchs)
