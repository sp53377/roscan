uint8 AC_NOT_ACTIVE = 0
uint8 AC_ACTIVE = 1
uint8 AC_ERROR = 2
uint8 AC_NOT_AVAILABLE = 3

uint8 D_REVERSE = 0
uint8 D_FORWARD = 1
uint8 D_ERROR = 2
uint8 D_NOT_AVAILABLE = 3

uint8 TCAS_SOLUTION_NA = 0
uint8 TCAS_LEVELING_IN_PROGRESS = 1
uint8 TCAS_YAW_ALIGNMENT_IN_PROGRESS = 2
uint8 TCAS_FINE_ALIGNMENT_IN_PROCESS = 3
uint8 TCAS_FINE_ALIGNMENT_COMPLETE = 4
uint8 TCAS_RESERVED1 = 5
uint8 TCAS_RESERVED2 = 6
uint8 TCAS_RESERVED3 = 7
uint8 TCAS_RESERVED4 = 8
uint8 TCAS_RESERVED5 = 9
uint8 TCAS_RESERVED6 = 10
uint8 TCAS_RESERVED7 = 11
uint8 TCAS_RESERVED8 = 12
uint8 TCAS_CALIBRATING = 13
uint8 TCAS_ERROR = 14
uint8 TCAS_NOT_AVAILABLE = 15

can_interfaces/CanData can_data

uint8 tc_mode
uint8 tc_accuracy
uint8 direction
uint8 course_direction
uint8 low_speed_capable
uint8 solution_projection_time_ms
bool is_valid
uint8 algorithm_status
