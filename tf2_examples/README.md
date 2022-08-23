# TF2 Python - WIP

This document and examples are meant as a cheat sheet of sort. You can reference the [official documentation](https://docs.ros.org/en/foxy/Tutorials/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html) for potentially more explanation. The source code can be found on the ROS2 Foxy [Geometry2 Gitub](https://github.com/ros2/geometry2/tree/foxy)


## TF2 Example Description
The [tf2_broadcaster](tf2_examples/tf2_broadcaster.py) node publishes transforms that are similar to the motion of the satellite seen in the GIF. While the [tf2_listener](tf2_examples/tf2_listener.py) node shows examples of looking up the transforms.

![satellite orbit](https://scijinks.gov/review/orbit/geo.gif)


## Command Line - Static Transform Publisher
Sometimes you may wish to create a static transform temporarly for development/debugging. The command below will create a tf2 transform that is static (position and orientation values will not change over time).
```bash
ros2 run tf2_ros static_transform_publisher x y z yaw pitch roll parent_frame child_frame
```

## Launch File - Static Transform Publisher
The Python snippet belows shows an example you could copy-paste into a launch file to add a static_transform_publisher. The ```name``` is arbitrary and should be changed to describe what this specific instance of the ```static_transform_publisher``` is doing. In this case it is being used to specfiy the transform that will take points expressed in ```lidar``` coordinate system and express them in the ```chassis``` coordinate system, hence ```lidar_to_chassis```.
```python
sample_name = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='lidar_to_chassis',
    arguments=["4.3434", "0", "0", "0", "0", "0", "chassis", "lidar"],
)
```
This example is the same as the previous but it has been configured to use a simulation time. This is common when using the Gazebo simulator.

```python
sample_name = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='lidar_to_chassis',
    arguments=["4.3434", "0", "0", "0", "0", "0", "chassis", "lidar"],
    parameters=[{'use_sim_time': True}]
)
```

# TF2 Incomplete Features - Foxy

* tf_transformation module does not work out of the box. It requires additional uninstalled dependencies. You will be informed of the dependency when you try to run a pythons script that attempts to import it.

* tf2_geometry_msgs for python does not work as of creation of this docmentation for python. This removes the use of "doTransform" functionality built in for the types listed below.

### Exhaustive doTransform List

* geometry_msgs/Point
* geometry_msgs/PointStamped
* sensor_msgs/PointCloud2
* geometry_msgs/Pose
* geometry_msgs/PoseStamped
* geometry_msgs/PoseWithCovariance
* geometry_msgs/PoseWithCovarianceStamped
* geometry_msgs/Quaternion
* geometry_msgs/QuaternionStamped
* geometry_msgs/Transform
* geometry_msgs/TransformStamped
* geometry_msgs/Wrench
* geometry_msgs/WrenchStamped