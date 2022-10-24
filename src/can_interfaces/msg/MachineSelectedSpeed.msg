uint8 D_REVERSE=0
uint8 D_FORWARD=1
uint8 D_ERROR=2
uint8 D_NOT_AVAILABLE=3

uint8 SPD_WHEEL = 0
uint8 SPD_GROUND = 1
uint8 SPD_GPS = 2
uint8 SPD_BLENDED = 3
uint8 SPD_SIMULATED = 4
uint8 SPD_RESERVED1 = 5
uint8 SPD_RESERVED2 = 6
uint8 SPD_NOT_AVAILABLE = 7

uint8 LS_NOT_LIMITED = 0
uint8 LS_OPERATOR_LIMITED = 1
uint8 LS_LIMITED_HIGH = 2
uint8 LS_LIMITED_LOW = 3
uint8 LS_ERROR = 6
uint8 LS_NOT_AVAILABLE = 7

can_interfaces/CanData can_data

float32 speed_mps
float32 distance_m
uint8 exit_code
uint8 direction
uint8 speed_source
uint8 speed_limit_status
