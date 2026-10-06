std_msgs/Header header
bool detected                        # false = no person this frame; other fields are empty
int32 id
float32 confidence
geometry_msgs/Point[] keypoints
float32[] keypoint_confidences