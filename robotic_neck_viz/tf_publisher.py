# ---- TF ----
#https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html


from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class TFPublisher(object):
    """
    A dynamic transforms broadcaster for ROS2.
    """

    def __init__(self, node):
        self.node = node
        self.tf_broadcaster = TransformBroadcaster(self.node) 


    def pub_dynamic_transform(self, parent_frame, child_frame, translation=[0.0,0.0,0.0], rotation=[0.0,0.0,0.0,1.0]):
        t = self.create_tf(parent_frame, child_frame, translation, rotation)
        self.tf_broadcaster.sendTransform(t) 
    

    def create_tf(self, parent_frame, child_frame, translation=[0.0,0.0,0.0], rotation=[0.0,0.0,0.0,1.0]):
        t = TransformStamped()

        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1] 
        t.transform.translation.z = translation[2]

        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]

        return t
