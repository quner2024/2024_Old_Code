# This message contains client data sent to the referee system

std_msgs/Header header

# target position, in meter
float32 target_position_x
float32 target_position_y

# target robot id, see enum below
uint16 target_robot_id

uint16 TARGET_ROBOT_ID_RED1_HERO                       = 1
uint16 TARGET_ROBOT_ID_RED2_ENGINEER                   = 2
uint16 TARGET_ROBOT_ID_RED3_INFANTRY                   = 3
uint16 TARGET_ROBOT_ID_RED4_INFANTRY                   = 4
uint16 TARGET_ROBOT_ID_RED5_INFANTRY                   = 5
uint16 TARGET_ROBOT_ID_RED6_AERIAL                     = 6
uint16 TARGET_ROBOT_ID_RED7_SENTRY                     = 7
uint16 TARGET_ROBOT_ID_RED9_RADAR                      = 9
uint16 TARGET_ROBOT_ID_RED10_OUTPOST                   = 10
uint16 TARGET_ROBOT_ID_RED11_BASE                      = 11

uint16 TARGET_ROBOT_ID_BLUE1_HERO                      = 101
uint16 TARGET_ROBOT_ID_BLUE2_ENGINEER                  = 102
uint16 TARGET_ROBOT_ID_BLUE3_INFANTRY                  = 103
uint16 TARGET_ROBOT_ID_BLUE4_INFANTRY                  = 104
uint16 TARGET_ROBOT_ID_BLUE5_INFANTRY                  = 105
uint16 TARGET_ROBOT_ID_BLUE6_AERIAL                    = 106
uint16 TARGET_ROBOT_ID_BLUE7_SENTRY                    = 107
uint16 TARGET_ROBOT_ID_BLUE9_RADAR                     = 109
uint16 TARGET_ROBOT_ID_BLUE10_OUTPOST                  = 110
uint16 TARGET_ROBOT_ID_BLUE11_BASE                     = 111