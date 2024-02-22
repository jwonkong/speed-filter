# Same as derived_object_msgs/Object
std_msgs/Header header

nav_msgs/Odometry odometry
nav_msgs/Odometry anchor
brain_msgs/Size size
geometry_msgs/Polygon polygon
brain_msgs/Track track
nav_msgs/Odometry[] tracklet

uint8 classification
uint8 unknown=0
uint8 unknown_small=1
uint8 unknown_medium=2
uint8 unknown_big=3
uint8 pedestrian=4
uint8 bike=5
uint8 car=6
uint8 truck=7
uint8 motorcycle=8
uint8 other_vehicle=9
uint8 barrier=10
uint8 sign=11

uint8 level
# uint8 unknown=0
uint8 detected=1
uint8 tracked=2
uint8 fused=3
uint8 predicted=4

uint8 sensor

uint8 motion_state
# uint8 unknown=0
# uint8 stop=1
# uint8 stationary=2
# uint8 moving=3

uint8 stop_count

uint8 intent
# uint8 unknown=0
uint8 stop=1
uint8 stationary=2
uint8 moving=3
uint8 change_lane=4
uint8 low_acceleration=5
uint8 high_acceleration=6
uint8 low_deceleration=7
uint8 high_deceleration=8

geometry_msgs/Point[] geometry

Feature feature
