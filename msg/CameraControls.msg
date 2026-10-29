# One per camera node, latched on /camera_controls
std_msgs/Header header
string camera_name        # "ee1_cam"
string device_node        # "/dev/port_a1_cam"
string card               # VIDIOC_QUERYCAP card name
bool available            # false: topic source, sim, or device failed to open
string set_service        # "/ee1_cam_set_controls"
CameraControl[] controls
