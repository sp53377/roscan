uint8 D_REVERSE=0
uint8 D_FORWARD=1
uint8 D_ERROR=2
uint8 D_NOT_AVAILABLE=3

uint8 OO_OFF=0
uint8 OO_ON=1
uint8 OO_ERROR=2
uint8 OO_NOT_AVAILABLE=3

uint8 EN_DISABLE=0
uint8 EN_ENABLE=1
uint8 EN_ERROR=2
uint8 EN_NOT_AVAILABLE=3

can_interfaces/CanData can_data

float32 speed_mps
float32 distance_m
int32 maximum_power_time_min
uint8 machine_direction
uint8 key_switch
uint8 start_stop
uint8 operator_direction