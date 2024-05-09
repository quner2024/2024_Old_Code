# This message contains dart client command data received from the referee system

std_msgs/Header header

# launch status, see enum below
uint8 launch_status

uint8 LAUNCH_STATUS_OPENED          = 0
uint8 LAUNCH_STATUS_CLOSED          = 1
uint8 LAUNCH_STATUS_IN_ACTION       = 2

# attack target, see enum below
uint8 attack_target

uint8 ATTACK_TARGET_OUTPOST         = 0
uint8 ATTACK_TARGET_BASE            = 1

# the remaining time of the game when switching targets, in seconds
uint16 target_change_time

# the remaining time of the game when the operator confirm to launch, in seconds
uint16 operate_launch_cmd_time
