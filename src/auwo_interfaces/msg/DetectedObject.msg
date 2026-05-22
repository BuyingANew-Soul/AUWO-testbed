# A single object detected by the perception pipeline
std_msgs/Header header
string label            # object class name e.g. "red_cube"
float32 confidence      # detection confidence 0.0 - 1.0
geometry_msgs/PoseStamped pose  # 3D pose in camera optical frame
int32 tracking_id       # stable ID across frames (-1 if not tracked)