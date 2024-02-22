std_msgs/Header header 

uint8 type

# base: grid map type 
float32 resolution
uint32 width
uint32 height
geometry_msgs/Pose origin
int8[] data

# points type
FreespacePoint[] points

# polygon type
geometry_msgs/Polygon outer
geometry_msgs/Polygon[] inners