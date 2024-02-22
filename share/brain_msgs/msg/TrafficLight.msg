# This message defines meta information for a traffic light status
std_msgs/Header header

# The result as provided by the tlr nodes
# Red and Yellow states are both treated as STOP
uint8 state
uint8 unknown    = 0
uint8 red        = 1
uint8 yellow     = 2
uint8 green      = 3
uint8 green_left = 4
uint8 red_left   = 5

# Probability of state [0,1]
# if is_its is true, it is set to be 1
float32 state_probability

# type
uint8 type
# uint8 UNKNOWN = 0
uint8 str       = 1
uint8 left      = 2
uint8 ped       = 3
uint8 uturn     = 4


## --------------------
## ITS ONLY INFORMATION
# true when it comes from ITS
bool is_its

# available when is_its is true
int64 intersection_id

# ID is avalialbe, if is_its is true
int64 group_id

# remaining time to be changed
float32 end_time # [sec]
## --------------------


## --------------------
## INFORMATION FROM DETECTION AND TRACKING
# Perception procssing level
uint8 level
uint8 detected=1
uint8 tracked=2
uint8 fused=3
uint8 predicted=4

# ID is avaliable, if perception level is tracking
int64 track_id

# Positional information of the traffic light information
geometry_msgs/PoseWithCovariance pose
## --------------------