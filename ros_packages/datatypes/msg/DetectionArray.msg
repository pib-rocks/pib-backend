# Detections for one model on one camera frame.
# header.stamp is for latency measurement. Coordinates refer to frame_width x frame_height.
std_msgs/Header header
string model_id         # e.g. "hand_landmark_224x224"
uint32 frame_width      # pixel size of the frame the coordinates refer to
uint32 frame_height
Detection[] detections
