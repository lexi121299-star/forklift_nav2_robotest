std_msgs/Header header

uint8 TYPE_NONE=0
uint8 TYPE_NAVIGATE=1
uint8 TYPE_PICKUP=2
uint8 TYPE_DROPOFF=3

uint8 STATUS_IDLE=0
uint8 STATUS_ACCEPTED=1
uint8 STATUS_RUNNING=2
uint8 STATUS_PAUSED=3
uint8 STATUS_SUCCEEDED=4
uint8 STATUS_FAILED=5
uint8 STATUS_CANCELED=6
uint8 STATUS_REJECTED=7

# Stable identifier supplied by dispatch. Legacy RViz requests receive a
# generated identifier and are also reported through this message.
string task_id
uint8 task_type
uint8 status

# Workflow-specific phase, for example NAVIGATING, RAISE_TO_SCAN_HEIGHT or
# INSERT_FORK. Progress is normalized to [0.0, 1.0] when known.
string phase
float32 progress

# Original requested slot or an empty string for pose navigation.
string slot_id
string reason
