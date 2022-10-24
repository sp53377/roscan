uint8 PM_UNDEFINED = 0
uint8 PM_NO_FIX = 1
uint8 PM_2D = 2
uint8 PM_3D = 3

uint8 CS_NO_FIX = 0
uint8 CS_FIX_NO_GNSS = 1
uint8 CS_FIX_WITH_DIFFERENTIAL = 2
uint8 CS_RESERVED1 = 3
uint8 CS_RESERVED2 = 4
uint8 CS_RESERVED3 = 5
uint8 CS_RESERVED4 = 6
uint8 CS_RESERVED5 = 7
uint8 CS_RESERVED6 = 8
uint8 CS_RESERVED7 = 9
uint8 CS_RESERVED8 = 10
uint8 CS_RESERVED9 = 11
uint8 CS_RESERVED10 = 12
uint8 CS_RESERVED11 = 13
uint8 CS_ERROR = 14
uint8 CS_NOT_AVAILABLE = 15

can_interfaces/CanData can_data

uint8 mode
uint8 state
uint8 velocity_solution_satellites
uint8 pdop
uint8 hdop
uint8 vdop
int32 correction_age_sec
