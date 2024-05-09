# This message contains interactive data sent to or received from the referee system

std_msgs/Header header

# data cmd id, see enum below
uint16 data_cmd_id

uint16 DATA_CMD_ID_ROBOT_INTERACTIVE_BEGIN      = 512    #0x200
uint16 DATA_CMD_ID_ROBOT_INTERACTIVE_END        = 767    #0x2FF
uint16 DATA_CMD_ID_CLIENT_DELETE_GRAPHICS       = 256    #0x100
uint16 DATA_CMD_ID_CLIENT_DRAW_GRAPHICS_1       = 257    #0x101
uint16 DATA_CMD_ID_CLIENT_DRAW_GRAPHICS_3       = 258    #0x102
uint16 DATA_CMD_ID_CLIENT_DRAW_GRAPHICS_5       = 259    #0x103
uint16 DATA_CMD_ID_CLIENT_DRAW_GRAPHICS_7       = 260    #0x104
uint16 DATA_CMD_ID_CLIENT_DRAW_CHARACTER        = 272    #0x110

# sender id and receiver id, see enum below
uint16 sender_id
uint16 receiver_id

uint16 ROBOT_ID_RED1_HERO                       = 1
uint16 ROBOT_ID_RED2_ENGINEER                   = 2
uint16 ROBOT_ID_RED3_INFANTRY                   = 3
uint16 ROBOT_ID_RED4_INFANTRY                   = 4
uint16 ROBOT_ID_RED5_INFANTRY                   = 5
uint16 ROBOT_ID_RED6_AERIAL                     = 6
uint16 ROBOT_ID_RED7_SENTRY                     = 7
uint16 ROBOT_ID_RED9_RADAR                      = 9

uint16 ROBOT_ID_BLUE1_HERO                      = 101
uint16 ROBOT_ID_BLUE2_ENGINEER                  = 102
uint16 ROBOT_ID_BLUE3_INFANTRY                  = 103
uint16 ROBOT_ID_BLUE4_INFANTRY                  = 104
uint16 ROBOT_ID_BLUE5_INFANTRY                  = 105
uint16 ROBOT_ID_BLUE6_AERIAL                    = 106
uint16 ROBOT_ID_BLUE7_SENTRY                    = 107
uint16 ROBOT_ID_BLUE9_RADAR                     = 109

uint16 CLIENT_ID_RED1_HERO                      = 257    #0x101
uint16 CLIENT_ID_RED2_ENGINEER                  = 258    #0x102
uint16 CLIENT_ID_RED3_INFANTRY                  = 259    #0x103
uint16 CLIENT_ID_RED4_INFANTRY                  = 260    #0x104
uint16 CLIENT_ID_RED5_INFANTRY                  = 261    #0x105
uint16 CLIENT_ID_RED6_AERIAL                    = 262    #0x106

uint16 CLIENT_ID_BLUE1_HERO                     = 357    #0x165
uint16 CLIENT_ID_BLUE2_ENGINEER                 = 358    #0x166
uint16 CLIENT_ID_BLUE3_INFANTRY                 = 359    #0x167
uint16 CLIENT_ID_BLUE4_INFANTRY                 = 360    #0x168
uint16 CLIENT_ID_BLUE5_INFANTRY                 = 361    #0x169
uint16 CLIENT_ID_BLUE6_AERIAL                   = 362    #0x16A

# data length
uint8 data_length

# payload
uint8[] data
