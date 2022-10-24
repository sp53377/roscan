uint8 D_REVERSE=0
uint8 D_FORWARD=1
uint8 D_ERROR=2
uint8 D_NOT_AVAILABLE=3

uint8 LO_NOT_LOCKED = 0
uint8 LO_LOCKED = 1
uint8 LO_ERROR = 2
uint8 LO_NOT_AVAILABLE = 3

can_interfaces/CanData can_data

float64 latitude
float64 longitude
float64 elevation_m
uint8 accuracy
float32 cos_roll_angle_rad
float32 roll_angle_deg
float32 yaw_rate_dps
float32 pitch_deg
float32 altitude_m
float32 speed_kph
float32 bearing_deg
uint8 direction
uint8 diff_locked
uint8 pdop
uint8 hdop
uint8 vdop
uint32 serial_number
