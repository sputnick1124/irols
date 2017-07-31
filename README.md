*IROLS*
**Intelligent RObotic Landing System**
This package is intended for use with a ROS-controlled
quadrotor to allow it to land on a moving platform. This
is done with a visual feedback from a camera. Heavy
emphasis was placed on simulation in Gazebo with the gazebo_ros
package. Everything is tested on my desktop with Gazebo 6.6.0.

The quadcopter model is basiacally a modified 3DR Iris model from [PX4](http://github.com/PX4/Firmware/Tools/sitl_gazebo/model/iris)
with some small modifications. The rover is taken from [gazebosim.org](http://models.gazebosim.org/pioneer3at)
again with some modifications (changed friction model, added landing platform,
attached ros skid steer controller, etc).

Dependencies for running the simulation include ROS, Gazebo7, [PX4Firmware](https://github.com/px4/Firmware).

**Run simulation**
`source scripts/setup_gazebo_ros.bash /abs/path/to/PX4/Firmware`
...
Lots of other things. Among them, `irols_landing.launch`,`irols.launch`,`action.launch`,`ekf.launch`,`commander.py`

Rover control is exerted at `/p3at/cmd_vel` with a `geometry_msgs/Twist` msg. The `pioneer_*.py` nodes take care of moving the rover. NB: in some installations of Gazebo, the rover has faulty inertial settings (I think) and it flips over when the wheels move. Good luck.
The downward-facing camera is exposed in the namespace `/camera1`.
