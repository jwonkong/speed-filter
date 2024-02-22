int64                 id
int64                 intersection_id
int64                 group_id

uint8                 type
uint8                 traffic_sign=0
uint8                 traffic_light=1
uint8                 speed_limit=2
uint8                 right_of_way=3
uint8                 all_way_stop=4

uint8                 subtype

uint8                 traffic_sign_unknown=0
uint8                 traffic_sign_school_zone=1
uint8                 traffic_sign_speed_bump=2

uint8                 traffic_light_unknown=0
uint8                 traffic_light_red=1
uint8                 traffic_light_red_left=2
uint8                 traffic_light_yellow=3
uint8                 traffic_light_green=4
uint8                 traffic_light_green_left=5

uint8                 speed_limit_5=5
uint8                 speed_limit_10=10
uint8                 speed_limit_15=15
uint8                 speed_limit_20=20
uint8                 speed_limit_30=30
uint8                 speed_limit_40=40
uint8                 speed_limit_50=50
uint8                 speed_limit_60=60
uint8                 speed_limit_70=70
uint8                 speed_limit_80=80
uint8                 speed_limit_90=90
uint8                 speed_limit_100=100

float32               remaining_time

geometry_msgs/Point[] refers
geometry_msgs/Point[] ref_line
geometry_msgs/Point[] cancels
geometry_msgs/Point[] cancel_line