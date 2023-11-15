# ---- Parameters ---- 
#https://docs.ros.org/en/humble/Concepts/Basic/About-Parameters.html
#https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html

# ---- Parameters Callback ----
#https://roboticsbackend.com/ros2-rclpy-parameter-callback/

import numpy as np
import rclpy


from rclpy.node import Node
from rclpy.parameter import ParameterType

from PyKDL import Chain, Segment, JntArray, Joint, Frame, Vector, ChainFkSolverPos_recursive

from tf_transformations import quaternion_from_euler, euler_from_quaternion
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult, IntegerRange

from sensor_msgs.msg import JointState

class NeckJointPublisher(Node):
    """
    This class is used to publish the joint states of the Robotic Neck
    """
    def __init__(self):
        super().__init__('neck_Joint_publisher')
        self.set_params()
        self.config_params()
        self.add_on_set_parameters_callback(self.parameters_callback)
       
        self.set_chains()
        
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)


    def set_params(self):
        self.roll = 0.0
        self.pitch = 0.0

        self.right_actuator_lentgh = 0.0
        self.right_actuator_roll = 0.0
        self.right_actuator_pitch = 0.0
        
        self.left_actuator_lentgh = 0.0
        self.left_actuator_roll = 0.0
        self.left_actuator_pitch = 0.0
        
        self.actuator_lentgh_init = 0.1645
        
        self.msg = JointState()
        self.msg.name = ["main_cross_piece_pitch_joint", "main_cross_piece_roll_joint",
                         "right_down_cross_piece_roll_joint", "right_down_cross_piece_pitch_joint",
                         "right_linear_actuator_tube_joint", "right_up_cross_piece_pitch_joint",
                         "right_up_cross_piece_roll_joint", "left_down_cross_piece_roll_joint",
                         "left_down_cross_piece_pitch_joint" ,"left_linear_actuator_tube_joint",
                         "left_up_cross_piece_pitch_joint", "left_up_cross_piece_roll_joint"]
        
        self.msg.position = [self.pitch, self.roll, 
                             self.right_actuator_roll, self.right_actuator_pitch, 
                             self.right_actuator_lentgh, 0.0, 
                             0.0, self.right_actuator_roll, 
                             self.right_actuator_pitch, self.left_actuator_lentgh, 
                             0.0, 0.0]


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
            
            if param.name == 'roll':
                self.roll = np.deg2rad(param.value)
            
            elif param.name == 'pitch':
                self.pitch = np.deg2rad(param.value)
        
        self.right_actuator_lentgh, self.right_actuator_roll, self.right_actuator_pitch = self.get_actuator_lentgh(self.chains["right"])
        self.left_actuator_lentgh, self.left_actuator_roll, self.left_actuator_pitch = self.get_actuator_lentgh(self.chains["left"])
        
        return SetParametersResult(successful=True)
    

    def set_chains(self):
        self.chains = {"right": {"vectors":[[0.075, 0.05, 0.166], [0, 0, 0], [-0.075, -0.05, -0.0015]],  # 0.166 is the z offset between the main cross piece and the right cross piece actuator base. = 172 + 11.5 - 13 - 4 - 0.5.
                                 "joints":[Joint.RotZ, Joint.RotY, Joint.RotX]},
                        "left": {"vectors":[[0.075, -0.05, 0.166], [0, 0, 0], [-0.075, 0.05, -0.0015]],
                                 "joints":[Joint.RotZ, Joint.RotY, Joint.RotX]}}
        
        self.define_chain(self.chains["right"])
        self.define_chain(self.chains["left"])
    
    
    def define_chain(self, chain):
        vectors, joints = chain["vectors"], chain["joints"]
        chain_obj = Chain()
       
        for vector, joint in zip(vectors, joints):
            segment = Segment(Joint(joint), Frame(Vector(*vector)))
            chain_obj.addSegment(segment)

        chain["chain"] = chain_obj
        chain["fk_solver"] = ChainFkSolverPos_recursive(chain_obj)
    

    def get_ralative_pose(self, fk_solver, roll, pitch):
    
        jointAngles = JntArray(3)
        
        jointAngles[0] = 0.0
        jointAngles[1] = pitch
        jointAngles[2] = roll 

        finalFrame = Frame()
        fk_solver.JntToCart(jointAngles,finalFrame)
        
        return finalFrame


    def get_actuator_lentgh(self, chain):
        
        frame = self.get_ralative_pose(chain["fk_solver"], self.roll, self.pitch)
        pos, R_matrix = frame.p, frame.M
        
        translation = np.array([pos[0], pos[1], pos[2]])
        lentgh = np.linalg.norm(translation)

        or_R, or_P, or_Y = translation/lentgh
        rot_roll, rot_pitch, rot_yaw = euler_from_quaternion(quaternion_from_euler(-or_P, or_R, 0))

        return lentgh - self.actuator_lentgh_init, rot_roll, rot_pitch


    def publish_joint_states(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()

        self.msg.position = [self.pitch, self.roll, 
                             self.right_actuator_roll, self.right_actuator_pitch, 
                             self.right_actuator_lentgh, 0.0, 
                             0.0, self.left_actuator_roll, 
                             self.left_actuator_pitch, self.left_actuator_lentgh, 
                             0.0, 0.0]
        
        self.joint_publisher.publish(self.msg)
        
        
def main():
    rclpy.init()
    node = NeckJointPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
