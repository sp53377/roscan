uint8 LO_NOT_LOCKED = 0
uint8 LO_LOCKED = 1
uint8 LO_ERROR = 2
uint8 LO_NOT_AVAILABLE = 3

uint8 DS_NONE = 0
uint8 DS_OTHER = 1
uint8 DS_JD = 2
uint8 DS_WAAS = 3
uint8 DS_SF1 = 4
uint8 DS_SF2 = 5
uint8 DS_RTKX = 6
uint8 DS_RTK = 7
uint8 DS_RESERVED0 = 8
uint8 DS_RESERVED1 = 9
uint8 DS_RESERVED2 = 10
uint8 DS_RESERVED3 = 11
uint8 DS_RESERVED4 = 12
uint8 DS_RESERVED5 = 13
uint8 DX_ERROR = 14
uint8 DS_NOT_AVAILABLE = 15

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

uint8 CO_NOT_CORRECT = 0
uint8 CO_CORRECT = 1
uint8 CO_ERROR = 2
uint8 CO_NOT_AVAILABLE = 3

uint8 SS_OUT_OF_RANGE = 0
uint8 SS_NO_FAILURE = 1
uint8 SS_NOT_ENOUGH_GPS_MEASUREMENTS_TO_INITIALIZE = 2
uint8 SS_NO_INITIAL_POSITION =3
uint8 SS_IN_PROGRESS =4
uint8 SS_SS_NOT_ENOUGH_GPS_MEASUREMENTS_TO_CONTINUE =5
uint8 SS_LOW_PDOP =6
uint8 SS_NO_VELOCITY_SOLUTION =7
uint8 SS_UPDATE_TO_LARGE = 8
uint8 SS_HIGH_VELOCITY_LIMITS_EXCEEDED =9
uint8 SS_DISABLED_OR_NOT_AUTHORIZED = 10
uint8 SS_MISSING_PVT = 11
uint8 SS_RESERVED1 = 12
uint8 SS_RESERVED2 = 13
uint8 SS_ERROR =14
uint8 SS_NOT_AVAILABLE = 15

uint8 AC_NOT_ACTIVE = 0
uint8 AC_ACTIVE = 1
uint8 AC_ERROR = 2
uint8 AC_NOT_AVAILABLE = 3

can_interfaces/CanData can_data

uint8 navigation_locked
int32 activation_delta_sec
uint8 navigation_mode
uint8 velocity_solution_satellites
uint8 state
uint32 satellites_mask
uint8 tc_accuracy
float32 signal_to_noise_ratio_db
float32 rtk_distance_km
float32 altitude_error_mm
float32 pitch_deg
float32 altitude_m
float32 bearing_deg
float32 speed_kph
int32 solution_projection_time_ms
int32 correction_age_sec
float32 horizontal_error_estimate_cm
uint8 sequence
uint8 decode
uint8 solution_status
uint8 low_speed_capable
uint8 source
uint32 serial_number