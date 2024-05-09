# This message contains robot hurt data received from the referee system

std_msgs/Header header

# hurt type, see enum below
uint8 hurt_type

uint8 HURT_TYPE_ARMOR_DAMAGE       = 0
uint8 HURT_TYPE_MODULE_OFFLINE     = 1
uint8 HURT_TYPE_SHOOT_OVER_SPEED   = 2
uint8 HURT_TYPE_SHOOT_OVER_HEAT    = 3
uint8 HURT_TYPE_CHASSIS_OVER_POWER = 4
uint8 HURT_TYPE_ARMOR_COLLISION    = 5

# armor id
uint8 armor_id
