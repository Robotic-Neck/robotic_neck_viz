
# https://github.com/ros/geometry2/blob/noetic-devel/tf2_ros/src/tf2_ros/buffer.py
# https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html
# https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Learning-About-Tf2-And-Time-Py.html

import numpy as np
import rclpy

from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from std_msgs.msg import Float64
from tf_transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix, quaternion_multiply
from geometry_msgs.msg import TransformStamped

from tf_publisher import TFPublisher

class TensorLenSim(Node):

    def __init__(self):
        super().__init__('tensor_lenth_simulator')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self) 
        self.timer = self.create_timer(0.1, self.publish_tensor_transforms)
        self.tf_publisher = TFPublisher(self)
        self.right_tensor_pub = self.create_publisher(Float64, 'right_tensor_len', 10)
        self.left_tensor_pub = self.create_publisher(Float64, 'left_tensor_len', 10)


    def publish_tensor_transforms(self):
        
        right_len = self.pub_tensor_transform('right_up_tensor', 'right_down_tensor', 'right_down_tensor_projection')
        left_len = self.pub_tensor_transform('left_up_tensor', 'left_down_tensor', 'left_down_tensor_projection')
        
        self.right_tensor_pub.publish(Float64(data=right_len))
        self.left_tensor_pub.publish(Float64(data=left_len))


    def get_tf(self, source_frame, target_frame, timeout=0.0):
        tf = TransformStamped()
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame=target_frame,
                source_frame=source_frame,
                time=self.get_clock().now()- rclpy.time.Duration(seconds=0.25))# 0.25 seconds ago
                #timeout=Duration(seconds=timeout))
            
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {source_frame} to {target_frame}: {ex}')
        
        return tf    


    def pub_tensor_transform(self, from_frame, to_frame, frame_name):

        from_tf = self.get_tf(from_frame, 'world')
        to_tf = self.get_tf(to_frame, 'world')

        delta_x = to_tf.transform.translation.x - from_tf.transform.translation.x
        delta_y = to_tf.transform.translation.y - from_tf.transform.translation.y
        delta_z = to_tf.transform.translation.z - from_tf.transform.translation.z
        
        translation = np.array([delta_x, delta_y, delta_z, 0])
        
        lentgh = np.linalg.norm(translation)

        or_R, or_P, or_Y = translation[:3]/lentgh

        from_tf_R, from_tf_P, from_tf_Y = euler_from_quaternion([from_tf.transform.rotation.x, from_tf.transform.rotation.y, from_tf.transform.rotation.z, from_tf.transform.rotation.w])
        to_tf_R, to_tf_P, to_tf_Y = euler_from_quaternion([to_tf.transform.rotation.x, to_tf.transform.rotation.y, to_tf.transform.rotation.z, to_tf.transform.rotation.w])
        d_R, d_P, d_Y = to_tf_R - from_tf_R, to_tf_P - from_tf_P, to_tf_Y - from_tf_Y
        
        d_q = quaternion_from_euler(d_R, d_P, d_Y)
    
        R_matrix = quaternion_matrix(d_q)
        
        dx, dy, dz = np.dot(R_matrix, translation)[:3]
        
        q_rotation = quaternion_multiply(d_q, quaternion_from_euler(or_P, -or_R, 0))
    
        self.tf_publisher.pub_dynamic_transform(from_frame, frame_name, translation=[dx, dy, dz], rotation=q_rotation)

        return lentgh

def main():
    rclpy.init()
    node = TensorLenSim()
    rclpy.spin(node)
    rclpy.shutdown()
