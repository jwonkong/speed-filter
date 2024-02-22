std_msgs/Header       header 
brain_msgs/PathNode[] nodes

uint8 behavior
uint8 unknown=0
uint8 onroad=1
uint8 parking=2
uint8 emergency=3

uint8 subtype
uint8 lane_keeping=1
uint8 lane_keeping_left_turn=2
uint8 lane_keeping_right_turn=3
uint8 lane_keeping_u_turn=4
uint8 change_left=5
uint8 change_right=6
uint8 approaching_parking=7
uint8 perpendicular_parking=8
uint8 parallel_parking=9
uint8 angular_parking=10
uint8 pull_over=11
uint8 return_to_lane=12
uint8 pull_over_and_stop=13
uint8 in_lane_stop=14

uint8 state
uint8 ready=0
uint8 ongoing=1
uint8 complete=2