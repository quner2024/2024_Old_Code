# This message contains referee warning data received from the referee system

std_msgs/Header header

# warning level, see enum below
uint8 level

uint8 LEVEL_YELLOW                  = 0
uint8 LEVEL_RED                     = 0
uint8 LEVEL_LOSE_GAME               = 0

# foul robot id, see enum below
uint8 foul_robot_id

uint8 FOUL_ROBOT_ID_ALL             = 0

uint8 FOUL_ROBOT_ID_RED1_HERO       = 1
uint8 FOUL_ROBOT_ID_RED2_ENGINEER   = 2
uint8 FOUL_ROBOT_ID_RED3_INFANTRY   = 3
uint8 FOUL_ROBOT_ID_RED4_INFANTRY   = 4
uint8 FOUL_ROBOT_ID_RED5_INFANTRY   = 5
uint8 FOUL_ROBOT_ID_RED6_AERIAL     = 6
uint8 FOUL_ROBOT_ID_RED7_SENTRY     = 7

uint8 FOUL_ROBOT_ID_BLUE1_HERO      = 101
uint8 FOUL_ROBOT_ID_BLUE2_ENGINEER  = 102
uint8 FOUL_ROBOT_ID_BLUE3_INFANTRY  = 103
uint8 FOUL_ROBOT_ID_BLUE4_INFANTRY  = 104
uint8 FOUL_ROBOT_ID_BLUE5_INFANTRY  = 105
uint8 FOUL_ROBOT_ID_BLUE6_AERIAL    = 106
uint8 FOUL_ROBOT_ID_BLUE7_SENTRY    = 107
