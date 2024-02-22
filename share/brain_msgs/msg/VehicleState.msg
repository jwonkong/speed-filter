std_msgs/Header                 header 

# odometry
nav_msgs/Odometry               odometry
brain_msgs/Wgs84                gnss
brain_msgs/Wgs84                reference

# chassis
float32                         speed
float32                         longitudinal_acceleration
float32                         lateral_acceleration
float32                         steering_angle
brain_msgs/Gear                 gear

# body
brain_msgs/TurnSignal           turn_signal
brain_msgs/MultiFunctionSwitch  mfs