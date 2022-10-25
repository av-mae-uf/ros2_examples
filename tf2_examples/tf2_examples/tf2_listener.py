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
                Lookup the transfrom between "earth" and "satellite", i.e one that will take 
                points expressed in the satellite's coordinates and expressed them in earth's coordinates,
                at the specified time (0 means current time). A timeout is specified so that the lookup will fail 
                if a transform cannot be found within the specified time.
                An object of type geometry_msgs.msg.TransformStamped is returned if successful.
            """
            satellite_to_earth_transform = self.tf2_buff.lookup_transform(target_frame="earth", source_frame="satellite",
                                                             time=tf2_ros.Time(seconds=0),
                                                             timeout=tf2_ros.Duration(seconds=0.2))
        except tf2_ros.TransformException as ex:
            # Catch the error if one occurs when looking up the transform
            self.get_logger().warn('{}: {}'.format(type(ex).__name__, ex))
            return
        
        try:
            """
                You can also lookup the inverse transform. This will return the transform that will take points
                expressed in the "earth" coordinates and express them in "satellite" coordinates
            """
            earth_to_satellite_transform = self.tf2_buff.lookup_transform(target_frame="satellite", source_frame="earth",
                                                             time=tf2_ros.Time(seconds=0),
                                                             timeout=tf2_ros.Duration(seconds=0.2))
        except tf2_ros.TransformException as ex:
            # Catch the error if one occurs when looking up the transform
            self.get_logger().warn('{}: {}'.format(type(ex).__name__, ex))
            return

        self.get_logger().info(f"{repr(earth_to_satellite_transform)}")


def main(args=None):
    rclpy.init(args=args)

    tf2_listener = TF2Listener()

    rclpy.spin(tf2_listener)

    tf2_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
