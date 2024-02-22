int64                           id

uint8                           type
uint8                           unknown=0
uint8                           parking=1
uint8                           freespace=2
uint8                           vegetation=3
uint8                           building=4
uint8                           traffic_island=5
uint8                           exit=6

geometry_msgs/Polygon           outer
geometry_msgs/Polygon[]         inners
brain_msgs/RegulatoryElement[]  regulatory_elements
brain_msgs/Relation[]           relation_from
brain_msgs/Relation[]           relation_to