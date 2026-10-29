# Matches V4L2_CTRL_TYPE_*
uint8 TYPE_INTEGER=1
uint8 TYPE_BOOLEAN=2
uint8 TYPE_MENU=3
uint8 TYPE_BUTTON=4
uint8 TYPE_INTEGER_MENU=9

string name               # "exposure_time_absolute"
uint32 id                 # 0x009a0902
uint8 type
int64 minimum
int64 maximum
int64 step
int64 default_value
int64 current_value
bool read_only
bool inactive             # e.g. exposure while auto exposure is on
CameraControlMenuEntry[] menu  # menu types only
