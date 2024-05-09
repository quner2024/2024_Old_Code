# This message contains power and heat data received from the referee system

std_msgs/Header header

# chassis output voltage, in volt
float32 chassis_volt

# chassis output current, in amp
float32 chassis_current

# chassis current power consumption, in watt
float32 chassis_power

# chassis power buffer, in joule
uint16 chassis_power_buffer

# shooter heat
uint16 shooter_17mm_id1_heat
uint16 shooter_17mm_id2_heat
uint16 shooter_42mm_id1_heat
