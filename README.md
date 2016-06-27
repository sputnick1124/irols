*IROLS*
**Intelligent RObotic Landing System**
This package is intended for use with a ROS-controlled
quadrotor to allow it to land on a moving platform. This
is done with a visual feedback from a camera. Heavy
emphasis was placed on simulation in Gazebo with the gazebo_ros
package. Everything is tested on my desktop with Gazebo 6.6.0.

The quadcopter model is just the 3DR Iris model from [PX4](http://github.com/PX4/Firmware/Tools/sitl_gazebo/model/iris)
with some small modifications. The rover is taken from [gazebosim.org](http://models.gazebosim.org/pioneer3at)
again with some modifications (changed friction model, added landing platform,
attached ros skid steer controller, etc).

Dependencies for running the simulation include ROS, Gazebo6, PX4Firmware.

**Run simulation**
`source gazebo/setup_gazebo_ros.bash`
`roslaunch irols mavros_posix_sitl_irols.launch`

Rover control is exerted at `/p3at/cmd_vel` with a `geometry_msgs/Twist` msg.
The downward-facing camera is exposed in the namespace `/camera1`.
