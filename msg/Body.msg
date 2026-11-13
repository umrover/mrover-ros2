std_msgs/Header header
bool detected                        # false = no person this frame; other fields are empty
int32 id
float32 confidence
geometry_msgs/Point[] keypoints      # sl::BODY_FORMAT::BODY_18 order, in header.frame_id; NaN when not detected
geometry_msgs/Point[] keypoints_2d   # pixel (x=u, y=v, z unused) in the published left image; negative = not detected
float32[] keypoint_confidences
geometry_msgs/Point position         # body centroid from the ZED (no orientation: unavailable without body fitting / BODY_18)
