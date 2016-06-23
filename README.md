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
