# ROS2 Message
int32     INITIALIZING = 0
int32     TRACKING = 1
int32     LOST = 2

std_msgs/Header header
int32 tracking_status 
float32 tracking_time_elapsed_ms
float32 extraction_time_elapsed_ms
