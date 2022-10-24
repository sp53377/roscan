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

uint8 AC_NOT_ACTIVE = 0
uint8 AC_ACTIVE = 1
uint8 AC_ERROR = 2
uint8 AC_NOT_AVAILABLE = 3

uint16 AT_CAPABLE = 0x4154
uint16 AT_ACTIVE = 0x4750
uint16 AT_NOT_AVAILABLE = 0x0000

uint8 PM_UNDEFINED = 0
uint8 PM_NO_FIX = 1
uint8 PM_2D = 2
uint8 PM_3D = 3

uint8 RV_GEN2 = 0
uint8 RV_GEN4 = 1
uint8 RV_LOW_COST = 2
uint8 RV_GEN5 = 3
uint8 RV_GEN6 = 4
uint8 RV_SF7000 = 5
uint8 RV_RESERVED2 = 6
uint8 RV_RESERVED3 = 7
uint8 RV_RESERVED4 = 8
uint8 RV_RESERVED5 = 9
uint8 RV_RESERVED6 = 10
uint8 RV_RESERVED7 = 11
uint8 RV_RESERVED8 = 12
uint8 RV_RESERVED9 = 13
uint8 RV_ERROR = 14
uint8 RV_NOT_AVAILABLE = 15

uint8 RT_UNKNOWN = 0
uint8 RT_VEHICLE = 1
uint8 RT_IMPLEMENT = 2

can_interfaces/CanData can_data

uint8 diff_source
uint8 sf_active
uint8 sf2_active
uint8 rtk_active
uint8 dual_frequency
uint8 sf_activation
uint8 sf2_license
uint8 rtk_activation
uint16 at_capable
uint8 tc_mode
uint8 mode
uint16 license_days
uint8 hardware_version
uint8 software_version
uint32 serial_number
uint8 receiver_version
uint8 type
