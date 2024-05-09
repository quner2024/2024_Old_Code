# This message contains robot status data received from the referee system

std_msgs/Header header

# robot id, see enum below
uint8 robot_id

uint8 ROBOT_ID_RED1_HERO        = 1
uint8 ROBOT_ID_RED2_ENGINEER    = 2
uint8 ROBOT_ID_RED3_INFANTRY    = 3
uint8 ROBOT_ID_RED4_INFANTRY    = 4
uint8 ROBOT_ID_RED5_INFANTRY    = 5
uint8 ROBOT_ID_RED6_AERIAL      = 6
uint8 ROBOT_ID_RED7_SENTRY      = 7
uint8 ROBOT_ID_RED8_DART        = 8
uint8 ROBOT_ID_RED9_RADAR       = 9
uint8 ROBOT_ID_BLUE1_HERO       = 101
uint8 ROBOT_ID_BLUE2_ENGINEER   = 102
uint8 ROBOT_ID_BLUE3_INFANTRY   = 103
uint8 ROBOT_ID_BLUE4_INFANTRY   = 104
uint8 ROBOT_ID_BLUE5_INFANTRY   = 105
uint8 ROBOT_ID_BLUE6_AERIAL     = 106
uint8 ROBOT_ID_BLUE7_SENTRY     = 107
uint8 ROBOT_ID_BLUE8_DART       = 108
uint8 ROBOT_ID_BLUE9_RADAR      = 109

# robot level
uint8 robot_level

# robot remain hp
uint16 remain_hp

# robot max hp
uint16 max_hp

# 17mm shooter 1 data, speed limit in m/s
uint16 shooter_17mm_id1_cooling_rate
uint16 shooter_17mm_id1_cooling_limit
uint16 shooter_17mm_id1_speed_limit

# 17mm shooter 2 data, speed limit in m/s
uint16 shooter_17mm_id2_cooling_rate
uint16 shooter_17mm_id2_cooling_limit
uint16 shooter_17mm_id2_speed_limit

# 42mm shooter 1 data, speed limit in m/s
uint16 shooter_42mm_id1_cooling_rate
uint16 shooter_42mm_id1_cooling_limit
uint16 shooter_42mm_id1_speed_limit

# chassis power limit, in watt
uint16 chassis_power_limit

# power output status
bool gimbal_power_output
bool chassis_power_output
bool shooter_power_output
