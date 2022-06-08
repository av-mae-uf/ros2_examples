import rclpy
from rclpy.node import Node
import tf2_ros


class TF2Listener(Node):

    def __init__(self):
        super().__init__('tf2_listener')

        #---- Listener Setup ----
        self.tf2_buff = tf2_ros.Buffer() # TF2 requires an object to cache transforms.
        self.tf_lis = tf2_ros.TransformListener(buffer=self.tf2_buff, node=self)

        #---- Timer ----
        self.timer = self.create_timer(timer_period_sec=0.5, callback=self.timer_cb)

    def timer_cb(self)-> None:
        try:
            """
                Lookup a transfrom between world and point at the specified time (0 means current time0-).
                A timeout is specified so that the lookup will fail if a transform cannot 
                be found within the specified time.
                Returned type is geometry_msgs.msg.TransformStamped.
            """
            current_pose = self.tf2_buff.lookup_transform(target_frame="world", source_frame="point",
                                                             time=tf2_ros.Time(seconds=0),
                                                             timeout=tf2_ros.Duration(seconds=0.2))
        except Exception as ex:
            # Catch the error if one occurs when looking up the transform
            self.get_logger().warn('{}: {}'.format(type(ex).__name__, ex))
            return
        
        try:
            """
                You can also lookup the inverse transform. This result would return the a transform to take points
                expressed in the "world" coordinates and express them in "point" coordinates
            """
            inv_current_pose = self.tf2_buff.lookup_transform(target_frame="point", source_frame="world",
                                                             time=tf2_ros.Time(seconds=0),
                                                             timeout=tf2_ros.Duration(seconds=0.2))
        except Exception as ex:
            # Catch the error if one occurs when looking up the transform
            self.get_logger().warn('{}: {}'.format(type(ex).__name__, ex))
            return

        self.get_logger().info(f"{repr(inv_current_pose)}")


def main(args=None):
    rclpy.init(args=args)

    tf2_listener = TF2Listener()

    rclpy.spin(tf2_listener)

    tf2_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
