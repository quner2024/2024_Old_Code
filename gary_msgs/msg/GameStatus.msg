# This message contains game status data received from the referee system

std_msgs/Header header

# game type, see enum below
uint8 game_type

uint8 GAME_TYPE_RMUC            = 1
uint8 GAME_TYPE_RMUT            = 2
uint8 GAME_TYPE_RMUA            = 3
uint8 GAME_TYPE_RMUL3V3         = 4
uint8 GAME_TYPE_RMUL1V1         = 5


# game progress, see enum below
uint8 game_progress

uint8 PROGRESS_UNSTART          = 0
uint8 PROGRESS_PREPARE          = 1
uint8 PROGRESS_SELFCHECK        = 2
uint8 PROGRESS_5SCOUNTDOWN      = 3
uint8 PROGRESS_BATTLE           = 4
uint8 PROGRESS_CALCULATING      = 5

# the remaining time of the current phase, in seconds
uint16 stage_remain_time

# unix timestamp
uint64 sync_time_stamp
