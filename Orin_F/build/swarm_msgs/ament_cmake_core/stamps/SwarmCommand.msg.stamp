# GCS command — matches UDP command wire format

# Command constants
uint8 CMD_RTL=1
uint8 CMD_LAND=2
uint8 CMD_KILL=3
uint8 CMD_FOLLOW=4
uint8 CMD_HOVER=5
uint8 CMD_TAKEOFF=6
uint8 CMD_WASD=10
uint8 CMD_WAYPOINT=11
uint8 CMD_SPEED=12
uint8 CMD_ALTITUDE=13

uint8   cmd
float32 vn             # WASD north velocity (CMD_WASD)
float32 ve             # WASD east velocity (CMD_WASD)
float32 vd             # vertical velocity (CMD_ALTITUDE)
float64 lat            # waypoint latitude (CMD_WAYPOINT)
float64 lon            # waypoint longitude (CMD_WAYPOINT)
float32 speed          # speed setting (CMD_SPEED)
