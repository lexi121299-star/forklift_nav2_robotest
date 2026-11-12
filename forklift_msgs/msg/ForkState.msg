std_msgs/Header header

uint8 STATUS_IDLE=0
uint8 STATUS_ACCEPTED=1
uint8 STATUS_MOVING=2
uint8 STATUS_REACHED=3
uint8 STATUS_FAULT=4
uint8 STATUS_REJECTED=5

uint32 active_command_id
uint8 status

float64 height_m
float64 side_shift_m
float64 tilt_rad

bool height_valid
bool side_shift_valid
bool tilt_valid

bool height_lower_limit
bool height_upper_limit
bool side_shift_left_limit
bool side_shift_right_limit
bool tilt_forward_limit
bool tilt_backward_limit

uint32 fault_code
