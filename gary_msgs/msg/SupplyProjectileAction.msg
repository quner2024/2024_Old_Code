# This message contains supply projectile action data received from the referee system

std_msgs/Header header

# supply projectile id
uint8 supply_projectile_id

# supply robot id, see enum below
uint8 supply_robot_id

uint8 SUPPLY_ROBOT_NONE         = 0
uint8 SUPPLY_ROBOT_RED1         = 1
uint8 SUPPLY_ROBOT_RED2         = 2
uint8 SUPPLY_ROBOT_RED3         = 3
uint8 SUPPLY_ROBOT_RED4         = 4
uint8 SUPPLY_ROBOT_RED5         = 5
uint8 SUPPLY_ROBOT_BLUE1        = 101
uint8 SUPPLY_ROBOT_BLUE2        = 102
uint8 SUPPLY_ROBOT_BLUE3        = 103
uint8 SUPPLY_ROBOT_BLUE4        = 104
uint8 SUPPLY_ROBOT_BLUE5        = 105

# supply projectile step, see enum below
uint8 supply_projectile_step

uint8 SUPPLY_PROJECTILE_STEP_CLOSE   = 0
uint8 SUPPLY_PROJECTILE_STEP_PREPARE = 1
uint8 SUPPLY_PROJECTILE_STEP_FALLING = 2

# supply bullet number
uint8 supply_bullet_num
