# This message contains shoot data received from the referee system

std_msgs/Header header

# bullet type, see enum below
uint8 bullet_type

uint8 BULLET_TYPE_17MM      = 1
uint8 BULLET_TYPE_42MM      = 2

# shooter id, see enum below
uint8 shooter_id

uint8 SHOOTER_ID_17MM_ID1   = 1
uint8 SHOOTER_ID_17MM_ID2   = 2
uint8 SHOOTER_ID_42MM_ID1   = 3

# bullet freq, in hz
uint8 bullet_freq

# bullet speed, in m/s
float32 bullet_speed
