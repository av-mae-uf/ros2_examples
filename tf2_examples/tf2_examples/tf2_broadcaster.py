from typing import Tuple
from math import sqrt, pow, cos, sin, pi

import rclpy
from rclpy.node import Node
import tf2_ros
# import tf_transformations

from geometry_msgs.msg import TransformStamped, Vector3, Quaternion

    
def calculate_point_on_ellipse(theta: int, a: float, b: float)-> Tuple[float, float]:
    """ Will return a tuple containing the x, y coordinates of the point. """
    theta = theta * pi/180 # Convert to radians
    r = a*b / sqrt(pow(b*cos(theta),2) + pow(a*sin(theta),2))
    return r*cos(theta), r*sin(theta)


class TF2Broadcaster(Node):

    def __init__(self):
        super().__init__('tf2_broadcaster')

        # Create a transform broadcaster.
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(node=self)

        #---- Timer ----
        self.timer = self.create_timer(timer_period_sec=1.0, callback=self.timer_cb)

        #---- Variables ----
        self.theta = 0

    def timer_cb(self)-> None:
        """ Generate a transform every callback invocation """

        # Update angle. Wrap it around to 0 if it goes over 360.
        self.theta = (self.theta + 5) % 360 # Degrees

        x, y = calculate_point_on_ellipse(theta=self.theta, a=2.0, b=1.25)

        """
            Point coordinates as seen by an observer fixed to the "planar" coordinates.
        """
        ellipse_to_planar = TransformStamped()
        ellipse_to_planar.header.stamp = self.get_clock().now().to_msg()
        ellipse_to_planar.header.frame_id = 'planar'
        ellipse_to_planar.child_frame_id = 'point'
        ellipse_to_planar.transform.translation = Vector3(x=x, y=y, z=0.0)

        """
            Planar coordinates as seen by World. The plane that the ellipse lies on
            is rotated -45 degrees about the y-axis. A static_transform_publisher
            should most likely be used in place of this because the transform does
            not change over time. It is kept for an example of setting the rotation quaternion.
        """
        planar_to_world = TransformStamped()
        planar_to_world.header.stamp = self.get_clock().now().to_msg()
        planar_to_world.header.frame_id = 'world'
        planar_to_world.child_frame_id = 'planar'
        planar_to_world.transform.translation = Vector3(x=0.0, y=0.0, z=0.0)

        # Math for quaternion. Don't worry too much about this.
        quat_angle = -45/2 * pi/180 # convert to radians
        y_quat, w_quat = sin(quat_angle), cos(quat_angle)

        planar_to_world.transform.rotation = Quaternion(x=0.0, y=y_quat, z=0.0, w=w_quat)

        self.tf_broadcaster.sendTransform(transform=[ellipse_to_planar, planar_to_world])


def main(args=None):
    rclpy.init(args=args)

    tf2_broadcaster = TF2Broadcaster()

    rclpy.spin(tf2_broadcaster)

    tf2_broadcaster.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
