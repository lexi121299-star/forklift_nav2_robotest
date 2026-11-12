std_msgs/Header header

uint8 MODE_STOP=0
uint8 MODE_HEIGHT_POSITION=1
uint8 MODE_SIDE_SHIFT_POSITION=2
uint8 MODE_TILT_POSITION=3

# STOP immediately stops every active fork axis. Each POSITION mode requests
# one absolute target for the axis encoded directly by mode.
uint8 mode

# Incremented for every new position request. Retransmissions keep the same ID.
uint32 command_id

# Absolute target and speed limit in the calibrated coordinate system.
#
# MODE_HEIGHT_POSITION:     target is m,   max_speed is m/s.
# MODE_SIDE_SHIFT_POSITION: target is m,   max_speed is m/s.
# MODE_TILT_POSITION:       target is rad, max_speed is rad/s.
#
# max_speed=0 means the TC397 controller uses its safe default.
float64 target
float64 max_speed
