#https://wiki.ros.org/rviz/DisplayTypes/Marker
import numpy as np
import rclpy

from rclpy.node import Node

from visualization_msgs.msg import Marker

from std_msgs.msg import Header
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point


class LidarFOV(Node): 
    """
    Publish poligons msg to visualize the field of view of the lidar.
    """
    def __init__(self):
        super().__init__('lidar_fov')
        self.publisher = self.create_publisher(Marker, 'lidar_fov', 10)
        self.timer = self.create_timer(0.2, self.publish_marker)
    
    def publish_marker(self):
        msg = Marker()
        
        msg.header.frame_id = 'ouster_model_origin'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.ns = 'lidar_fov'

        msg.type = Marker.LINE_LIST
        msg.action = Marker.ADD

        msg.frame_locked = True
        msg.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        msg.scale.x = 0.008

        theta_range = np.linspace(-180, 135, 8)
        #phi_range = np.linspace(-45, 0, 32)
        phi_range = np.linspace(-15, 15, 16)
        
        for theta in theta_range:
            for phi in phi_range:
                self.add_line_proyection(msg.points, 0.0, 4.0, theta, phi)

        self.publisher.publish(msg)
    

    def add_line_proyection(self, points, r_min, r_max, theta, phi):
        points.append(self.get_point_proyection(r_min, theta, phi))
        points.append(self.get_point_proyection(r_max, theta, phi))
    

    def get_point_proyection(self, r, theta, phi):
        """
        r: distance to the point
        theta: deg angle in the horizontal plane
        phi: deg angle in the vertical plane
        """

        rad_theta = np.deg2rad(theta)
        rad_phi = np.deg2rad(phi)

        x = r*np.cos(rad_theta)*np.cos(rad_phi)
        y = r*np.sin(rad_theta)*np.cos(rad_phi)
        z = r*np.sin(rad_phi)
        
        return Point(x=x, y=y, z=z)

       
def main():
    rclpy.init()
    node = LidarFOV()
    rclpy.spin(node)
    rclpy.shutdown()