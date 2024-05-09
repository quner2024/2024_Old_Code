# This message contains game result data received from the referee system

std_msgs/Header header

# winner, see enum below
uint8 winner

uint8 WINNER_DRAW               = 0
uint8 WINNER_RED                = 1
uint8 WINNER_BLUE               = 2
