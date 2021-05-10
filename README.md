# Keyboard Teleoperation With PID Control

![Ground speed teleoperation control mode](https://i.ibb.co/m5ngjvQ/keyboardground.png)

![Pose teleoperation control mode](https://i.ibb.co/HGc47fV/keyboardpose.png)

![Attitude teleoperation control mode](https://i.ibb.co/hX7dkH2/keyboardattitude.png)

# Subscribed topics

- **self_localization/pose** ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))      
Current pose of the vehicle

- **self_localization/speed** ([geometry_msgs/TwistStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/TwistStamped.html))      
Current speed of the vehicle

- **motion_reference/speed** ([geometry_msgs/TwistStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/TwistStamped.html))  
Speed reference for the controller.

- **motion_reference/assumed_control_mode** ([aerostack_msgs/MotionControlMode](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/master/msg/MotionControlMode.msg))  
Current controller's control mode.

# Published topics

- **motion_reference/pose** ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))  
Pose reference for the controller.

- **motion_reference/speed** ([geometry_msgs/TwistStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/TwistStamped.html))  
Speed reference for the controller.

- **actuator_command/flight_action** ([aerostack_msgs/FlightActionCommand](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/7c07e4317e20a1142226d513336a06a2ff585629/msg/FlightActionCommand.msg))           
It changes the status of a quadrotor vehicle. 

---
# Contributors
**Code Maintainer:** Alberto Rodelgo Perales

**Author:** Alberto Rodelgo Perales