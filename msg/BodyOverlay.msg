# Operator-facing view of the tracked person, drawn by the camera client on top of the ZED video stream.
# Kept small on purpose since it goes to the base station over the radio: normalized 2D joints + gesture state.
std_msgs/Header header               # stamp = ZED image time of the body this was computed from

uint8 NO_BODY=0                      # nobody is being tracked
uint8 TRACKING=1                     # a person is tracked but not holding a T-pose
uint8 HOLDING=2                      # T-pose is being held, see hold_elapsed / hold_required
uint8 LOCKED=3                       # T-pose confirmed, the "astronaut" frame is being published to tf
uint8 state

float32 hold_elapsed                 # seconds the current T-pose has been held (HOLDING)
float32 hold_required                # seconds the T-pose must be held to lock

# BODY_18 joints (sl::BODY_18_PARTS order) in normalized image coordinates [0, 1] of the ZED left image.
# Negative = joint not detected. Empty when no body is tracked.
float32[] joints_u
float32[] joints_v
bool[] joints_confident              # joint passed the gesture node's min_joint_confidence
