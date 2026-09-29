# Generic detection: bbox plus optional named keypoints and scalars.
# Coordinates are pixels of the frame described by DetectionArray.
# Adding a model must not change this schema (parallel name/value arrays).
string label            # e.g. "person", "hand", "happy"
float32 score           # confidence 0..1
int32 x_min             # bounding box in pixels of the frame described by DetectionArray
int32 y_min
int32 x_max
int32 y_max
string[] keypoint_names # e.g. ["wrist","thumb_tip",...] / ["landmark_0",...]; empty for bbox-only detectors
float32[] keypoint_x    # pixels
float32[] keypoint_y    # pixels
float32[] keypoint_z    # millimetres, 0 = invalid / unavailable
string[] scalar_names   # e.g. ["yaw_deg","pitch_deg","roll_deg"]; empty if none
float32[] scalar_values
