# This message contains data received by the DJI DR16 receiver

std_msgs/Header header

# joysticks
float32 ch_left_x
float32 ch_left_y

float32 ch_right_x
float32 ch_right_y

float32 ch_wheel

# switch state definition
uint8 SW_UP = 1
uint8 SW_DOWN = 2
uint8 SW_MID = 3

# switch
uint8 sw_left
uint8 sw_right

# mouse
float32 mouse_x
float32 mouse_y
float32 mouse_z
bool mouse_press_l
bool mouse_press_r

# key board
bool key_w
bool key_s
bool key_a
bool key_d
bool key_shift
bool key_ctrl
bool key_q
bool key_e
bool key_r
bool key_f
bool key_g
bool key_z
bool key_x
bool key_c
bool key_v
bool key_b
