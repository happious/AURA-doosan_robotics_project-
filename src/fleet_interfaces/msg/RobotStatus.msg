string robot_id
uint8 mode
float64 battery_pct
geometry_msgs/PoseStamped pose
bool aed_loaded
bool available

uint8 MODE_IDLE=0
uint8 MODE_GUIDE=1
uint8 MODE_LUGGAGE_ASSIST=2
uint8 MODE_EMERGENCY_DISPATCH=3
uint8 MODE_ERROR=4
uint8 MODE_CHARGING=5
uint8 MODE_HOTPLACE_DISPATCH=6
