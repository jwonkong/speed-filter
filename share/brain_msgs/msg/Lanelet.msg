int64                           id

uint8                           type
uint8                           unknown=0
uint8                           road=1
uint8                           highway=2
uint8                           play_street=3
uint8                           emergency_lane=4
uint8                           bus_lane=5
uint8                           bicycle_lane=6
uint8                           walkway=7
uint8                           shared_walkway=8
uint8                           crosswalk=9
uint8                           stairs=10

geometry_msgs/Point[]           center_line
geometry_msgs/Point[]           left_boundary
geometry_msgs/Point[]           right_boundary
brain_msgs/RegulatoryElement[]  regulatory_elements

brain_msgs/Relation[]           relation_from 
brain_msgs/Relation[]           relation_to 