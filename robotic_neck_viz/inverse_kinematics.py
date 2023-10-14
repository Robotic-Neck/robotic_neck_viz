# http://wiki.ros.org/kdl/Tutorials/Frame%20transformations%20%28Python%29
# http://wiki.icub.org/wiki/KDL-simple
# http://docs.ros.org/en/diamondback/api/kdl/html/python/kinematic_chains.html#PyKDL.Segment
# https://roboticsbackend.com/ros2-global-parameters/
# sudo apt install python3-pykdl

import rclpy
import numpy as np

from PyKDL import Chain, Segment, JntArray, Joint, Frame, Vector, ChainFkSolverPos_recursive
from rclpy.node import Node
from tf_transformations import quaternion_from_euler
from neck_mec_sim.tf_publisher import TFPublisher

from rcl_interfaces.srv import GetParameters
from std_msgs.msg import Float64

class IK(Node):

    def __init__(self):
        super().__init__('inverse_kinematics')
        self.set_connections()
        self.set_chains()
        self.set_client()
        
        self.tf_publisher = TFPublisher(self)
        self.timer = self.create_timer(0.2, self.send_request)
    
    def set_connections(self):
        self.right_tensor_pub = self.create_publisher(Float64, 'right_tensor_len', 1)
        self.left_tensor_pub = self.create_publisher(Float64, 'left_tensor_len', 1)


    def set_chains(self):
        self.chains = {"right": {"vectors":[[0.075, 0.05, 0.1505], [0, 0, 0], [-0.075, -0.05, -0.0015]], 
                                 "joints":[Joint.RotZ, Joint.RotY, Joint.RotX]},
                        "left": {"vectors":[[0.075, -0.05, 0.1505], [0, 0, 0], [-0.075, 0.05, -0.0015]],
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


    def set_client(self):
        self.client = self.create_client(GetParameters, "/neck_mec_sim/get_parameters")
        self.client.wait_for_service()
        
        self.req = GetParameters.Request()
        self.req.names = ["roll", "pitch"]
    

    def send_request(self):
        future = self.client.call_async(self.req)
        future.add_done_callback(self.params_callback)
        

    def params_callback(self, future):
        try:
            result = future.result()
        
        except Exception as e:
            self.get_logger().warn("service call failed %r" % (e,))
        
        else:
            roll = np.deg2rad(result.values[0].integer_value)
            pitch = np.deg2rad(result.values[1].integer_value)
            
            r_len = self.pub_tensor_transform('right_down_tensor', 'right_up_tensor_projection', self.chains["right"], roll, pitch)
            l_len = self.pub_tensor_transform('left_down_tensor', 'left_up_tensor_projection', self.chains["left"], roll, pitch)
            
            self.right_tensor_pub.publish(Float64(data=r_len))
            self.left_tensor_pub.publish(Float64(data=l_len))

    
    def get_ralative_pose(self, fk_solver, roll, pitch):
    
        jointAngles = JntArray(3)
        
        jointAngles[0] = 0.0
        jointAngles[1] = pitch
        jointAngles[2] = roll 

        finalFrame = Frame()
        fk_solver.JntToCart(jointAngles,finalFrame)
        
        return finalFrame
    

    def pub_tensor_transform(self,from_frame, frame_name, chain, roll, pitch):
        
        frame = self.get_ralative_pose(chain["fk_solver"], roll, pitch)
        pos, R_matrix = frame.p, frame.M
        
        translation = np.array([pos[0], pos[1], pos[2]])
        lentgh = np.linalg.norm(translation)
        
        or_R, or_P, or_Y = translation/lentgh
        rotation = quaternion_from_euler(-or_P, or_R, 0)
       
        self.tf_publisher.pub_dynamic_transform(from_frame, frame_name, translation=translation, rotation=rotation)
        
        return lentgh


def main():
    rclpy.init()
    node = IK()
    rclpy.spin(node)
    rclpy.shutdown()