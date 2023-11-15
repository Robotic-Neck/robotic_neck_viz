# ---- Parameters ---- 
#https://docs.ros.org/en/humble/Concepts/Basic/About-Parameters.html
#https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html

# ---- Parameters Callback ----
#https://roboticsbackend.com/ros2-rclpy-parameter-callback/

import numpy as np
import rclpy

from rclpy.node import Node

from PyKDL import Chain, Segment, JntArray, Joint, Frame, Vector, ChainFkSolverPos_recursive

from tf_transformations import quaternion_from_euler, euler_from_quaternion

from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

class NeckStateJointPublisher(Node):
    """
    This class is used to publish the joint states of the Robotic Neck State using an IMU
    """
    def __init__(self):
        super().__init__('neck_state_joint_publisher')
        self.set_params()
        self.set_connections()
    
        self.set_chains()
        
        self.joint_publisher = self.create_publisher(JointState, 'state/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)

    def set_params(self):
        self.ns = "state"
        
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
        self.msg.name = [f"{self.ns}/main_cross_piece_pitch_joint",      f"{self.ns}/main_cross_piece_roll_joint",
                         f"{self.ns}/right_down_cross_piece_roll_joint", f"{self.ns}/right_down_cross_piece_pitch_joint",
                         f"{self.ns}/right_linear_actuator_tube_joint",  f"{self.ns}/right_up_cross_piece_pitch_joint",
                         f"{self.ns}/right_up_cross_piece_roll_joint",   f"{self.ns}/left_down_cross_piece_roll_joint",
                         f"{self.ns}/left_down_cross_piece_pitch_joint", f"{self.ns}/left_linear_actuator_tube_joint",
                         f"{self.ns}/left_up_cross_piece_pitch_joint",   f"{self.ns}/left_up_cross_piece_roll_joint"]
        
        self.msg.position = [self.pitch, self.roll, 
                             self.right_actuator_roll, self.right_actuator_pitch, 
                             self.right_actuator_lentgh, 0.0, 
                             0.0, self.right_actuator_roll, 
                             self.right_actuator_pitch, self.left_actuator_lentgh, 
                             0.0, 0.0]
    
    def set_connections(self):
        self.roll_sub = self.create_subscription(Float32, '/rpip/roll', self.roll_callback, 1)
        self.pitch_sub = self.create_subscription(Float32, '/rpip/pitch', self.pitch_callback, 1)
    
    def roll_callback(self, msg):
        self.roll = msg.data
    
    def pitch_callback(self, msg):
        self.pitch = msg.data   

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
        self.right_actuator_lentgh, self.right_actuator_roll, self.right_actuator_pitch = self.get_actuator_lentgh(self.chains["right"])
        self.left_actuator_lentgh, self.left_actuator_roll, self.left_actuator_pitch = self.get_actuator_lentgh(self.chains["left"])
        
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
    node = NeckStateJointPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
