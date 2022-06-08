# TF2 Python - WIP

This document and examples are meant as a cheat sheet of sort. You can reference the [official documentation](https://docs.ros.org/en/foxy/Tutorials/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html) for potentially more explanation. The source code can be found on the ROS2 Foxy [Geometry2 Gitub](https://github.com/ros2/geometry2/tree/foxy)


## TODO:
* Use tf_transformation module to show examples of converting a quaternion-rotation matrix, and vice versa.


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