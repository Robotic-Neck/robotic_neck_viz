# ---- TF ----
#https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html
#https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html

# ---- Parameters ---- 
#https://docs.ros.org/en/humble/Concepts/Basic/About-Parameters.html
#https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html

# ---- Parameters Callback ----
#https://roboticsbackend.com/ros2-rclpy-parameter-callback/

import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.parameter import ParameterType

from tf_transformations import quaternion_from_euler
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult, IntegerRange
from neck_mec_sim.tf_publisher import TFPublisher


class NeckMecSim(Node):
    """
    Broadcast transforms to create a neck mecanism simulation.
    """
    def __init__(self):
        super().__init__('neck_mechanism_simulator')
        self.set_params()
        self.config_params()
        self.add_on_set_parameters_callback(self.parameters_callback)
        self.tf_publisher = TFPublisher(self) 
        self.timer = self.create_timer(0.2, self.publish_dynamic_transforms)


    def set_params(self):
        self.roll = 0.0
        self.pitch = 0.0


    def config_params(self):
        roll_param_descriptor = ParameterDescriptor(
            name='roll',
            type=ParameterType.PARAMETER_DOUBLE,
            description='roll angle of the main universal joint',
            additional_constraints='only valid values are between -60 and 60',
            read_only=False,
            integer_range=[IntegerRange(from_value=-60, to_value=60, step=1)])
        self.declare_parameter(name="roll", value=0, descriptor=roll_param_descriptor)
       
        pitch_param_descriptor = ParameterDescriptor(
            name='pitch',
            type=ParameterType.PARAMETER_DOUBLE,
            description='roll angle of the main universal joint',
            additional_constraints='only valid values are between -60 and 60',
            read_only=False,
            integer_range=[IntegerRange(from_value=-60, to_value=60, step=1)])
        self.declare_parameter(name="pitch", value=0, descriptor=pitch_param_descriptor)


    def parameters_callback(self, params):
        for param in params:
            #print(vars(param))
            
            if param.name == 'roll':
                self.roll = np.deg2rad(param.value)
            
            elif param.name == 'pitch':
                self.pitch = np.deg2rad(param.value)
        
        return SetParametersResult(successful=True)

        
    def publish_dynamic_transforms(self):
        self.tf_publisher.pub_dynamic_transform('base_link', 'universal_joint', translation=[0.0,0.0,0.0115], rotation=quaternion_from_euler(self.roll, self.pitch, 0.0))


def main():
    rclpy.init()
    node = NeckMecSim()
    rclpy.spin(node)
    rclpy.shutdown()