# This message contains PID controller data

std_msgs/Header header

# parameters
float64 kp
float64 ki
float64 kd

# limitations
float64 max_out
float64 max_iout

# status
float64 set
float64 feedback
float64 error
float64 error_sum
float64 last_error

# output
float64 pout
float64 iout
float64 dout
float64 out
