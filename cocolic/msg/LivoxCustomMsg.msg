# Same layout as livox_ros_driver/CustomMsg (and livox_ros_driver2/msg/CustomMsg),
# so bags recorded with either driver can be deserialized without the Livox SDK.
std_msgs/Header header    # ROS standard message header
uint64 timebase           # The time of first point
uint32 point_num          # Total number of pointclouds
uint8  lidar_id           # Lidar device id number
uint8[3]  rsvd            # Reserved use
LivoxCustomPoint[] points # Pointcloud data
