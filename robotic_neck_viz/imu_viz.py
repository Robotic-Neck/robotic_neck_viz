# http://wiki.ros.org/kdl/Tutorials/Frame%20transformations%20%28Python%29
# http://wiki.icub.org/wiki/KDL-simple
# http://docs.ros.org/en/diamondback/api/kdl/html/python/kinematic_chains.html#PyKDL.Segment
# https://roboticsbackend.com/ros2-global-parameters/
# sudo apt install python3-pykdl

import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.parameter import Parameter

from tf_transformations import quaternion_from_euler, euler_from_quaternion

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterValue, ParameterType
from std_msgs.msg import Float32

class ImuViz(Node):

    def __init__(self):
        super().__init__('IMU_viz')
        self.set_params()
        self.set_connections()
        self.set_client()
        self.timer = self.create_timer(0.1, self.send_request)
    
    def set_params(self):
        self.roll = 0
        self.pitch = 0

    def set_connections(self):
        self.roll_sub = self.create_subscription(Float32, '/rpip/roll', self.roll_callback, 10)
        self.pitch_sub = self.create_subscription(Float32, '/rpip/pitch', self.pitch_callback, 10)

    def roll_callback(self, msg):
        self.roll = msg.data
    
    def pitch_callback(self, msg):
        self.pitch = msg.data

    def set_client(self):
        self.client = self.create_client(SetParameters, '/neck_joint_publisher/set_parameters')
        self.client.wait_for_service()
        self.req = SetParameters.Request()
    
    def send_request(self):
        print("Sending request")
        rotation = quaternion_from_euler(-self.pitch, self.roll, 0)
        roll, pitch, yaw = euler_from_quaternion(rotation)
        
        roll_value = ParameterValue(type=2, integer_value=15)
        pitch_value = ParameterValue(type=2, integer_value=0)
        
        self.req.parameters = [Parameter(name='roll', value=roll_value)]
        self.future = self.cli.call_async(self.req)

def main():
    rclpy.init()
    node = ImuViz()
    rclpy.spin(node)
    rclpy.shutdown()

main()