std_msgs/Header header

uint8 mode 
uint8 inactive=0
uint8 mode_acceleration=1
uint8 mode_torque=2
uint8 mode_pedal_position=3
uint8 mode_pedal_torque=4
uint8 mode_acc_pos_brk_torque=5
uint8 mode_acc_torque_brk_pos=6

float32 velocity
float32 acceleration
float32 torque
float32 acc_pos
float32 acc_torque
float32 brk_pos
float32 brk_torque 
float32 distance_error
float32 speed_error

brain_msgs/Gear gear