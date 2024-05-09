# This message contains dual-loop PID controller data

std_msgs/Header header

# parameters
float64 inner_kp
float64 inner_ki
float64 inner_kd
float64 inner_p_filter_coefficient
float64 inner_d_filter_coefficient

float64 outer_kp
float64 outer_ki
float64 outer_kd
float64 outer_p_filter_coefficient
float64 outer_d_filter_coefficient

# limitations
float64 inner_max_out
float64 inner_max_iout

float64 outer_max_out
float64 outer_max_iout

# status
float64 inner_set
float64 inner_feedback
float64 inner_error
float64 inner_error_sum
float64 inner_last_error

float64 outer_set
float64 outer_feedback
float64 outer_error
float64 outer_error_sum
float64 outer_last_error

# output
float64 inner_pout
float64 inner_iout
float64 inner_dout
float64 inner_out

float64 outer_pout
float64 outer_iout
float64 outer_dout
float64 outer_out
