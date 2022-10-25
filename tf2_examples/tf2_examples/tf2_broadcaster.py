from typing import Tuple
from math import sqrt, pow, cos, sin, pi

import rclpy
from rclpy.node import Node
import tf2_ros

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
        """ Generates a transform on every callback invocation """

        # Update angle. Wrap it around to 0 if it goes over 360.
        self.theta = (self.theta + 5) % 360 # Degrees

        x, y = calculate_point_on_ellipse(theta=self.theta, a=2.0, b=1.25)

        """
            The satellite's coordinates as seen by an observer fixed to the "orbital plane" coordinates.
        """
        satellite_to_orbital_plane = TransformStamped()
        satellite_to_orbital_plane.header.stamp = self.get_clock().now().to_msg()
        satellite_to_orbital_plane.header.frame_id = 'orbital_plane'
        satellite_to_orbital_plane.child_frame_id = 'satellite'
        satellite_to_orbital_plane.transform.translation = Vector3(x=x, y=y, z=0.0)

        """
            The "orbital plane" coordinates as seen by an observer on "earth". The orbital plane that the ellipse lies on
            is rotated -45 degrees about the y-axis. A static_transform_publisher
            should most likely be used in place of this because the transform does
            not change over time. It is kept for an example of setting the rotation quaternion.
        """
        orbital_plane_to_earth = TransformStamped()
        orbital_plane_to_earth.header.stamp = self.get_clock().now().to_msg()
        orbital_plane_to_earth.header.frame_id = 'earth'
        orbital_plane_to_earth.child_frame_id = 'orbital_plane'
        orbital_plane_to_earth.transform.translation = Vector3(x=0.0, y=0.0, z=0.0)

        # Math for quaternion. Don't worry too much about this.
        quat_angle = -45/2 * pi/180 # convert to radians
        y_quat, w_quat = sin(quat_angle), cos(quat_angle)

        orbital_plane_to_earth.transform.rotation = Quaternion(x=0.0, y=y_quat, z=0.0, w=w_quat)

        """ 
            It is possible to send multiple transforms in a single sendTransform call by 
            placing them in a list.
        """
        self.tf_broadcaster.sendTransform(transform=[satellite_to_orbital_plane, orbital_plane_to_earth])


def main(args=None):
    rclpy.init(args=args)

    tf2_broadcaster = TF2Broadcaster()

    rclpy.spin(tf2_broadcaster)

    tf2_broadcaster.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
