# This message contains field events data received from the referee system

std_msgs/Header header

# supply station occupation status
bool supplier_1_occupation
bool supplier_2_occupation
bool supplier_3_occupation

# power rune status
bool power_rune_activation_point_occupation
bool small_power_rune_activation_status
bool big_power_rune_activation_status

# trapezoid-shaped elevated ground status
bool r2b2_ground_occupation
bool r3b3_ground_occupation
bool r4b4_ground_occupation

# shield status of base
bool base_has_shield

# outpost status
bool outpost_alive
