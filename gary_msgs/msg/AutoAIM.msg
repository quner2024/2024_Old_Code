# This message contains auto aiming data

std_msgs/Header header

float32 pitch
float32 yaw

# auto aiming target, see enum below
uint8 target_id

uint8 TARGET_ID0_NONE                       = 0
uint8 TARGET_ID1_HERO                       = 1
uint8 TARGET_ID2_ENGINEER                   = 2
uint8 TARGET_ID3_INFANTRY                   = 3
uint8 TARGET_ID4_INFANTRY                   = 4
uint8 TARGET_ID5_INFANTRY                   = 5
uint8 TARGET_ID6_OUTPOST                    = 6
uint8 TARGET_ID7_SENTRY                     = 7

# auto aiming target distance
float32 target_distance

# auto aiming mode, see enum below
uint8 vision_mode

uint8 VISION_MODE_ARMOR                     = 1
uint8 VISION_MODE_SMALL                     = 2
uint8 VISION_MODE_BIG                       = 3
uint8 VISION_MODE_TAG                       = 4
