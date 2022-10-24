uint8 LO_NOT_LOCKED=0
uint8 LO_LOCKED=1
uint8 LO_ERROR=2
uint8 LO_NOT_AVAILABLE=3

uint8 AUT_WAAS=0
uint8 AUT_EGNOS=1
uint8 AUT_MSAS=2
uint8 AUT_GAGAN=3
uint8 AUT_SNAS=4
uint8 AUT_WADGPS=5
uint8 AUT_SDCM=6
uint8 AUT_RESERVED1=7
uint8 AUT_RESERVED2=8
uint8 AUT_RESERVED3=9
uint8 AUT_RESERVED4=10
uint8 AUT_RESERVED5=11
uint8 AUT_RESERVED6=12
uint8 AUT_RESERVED7=13
uint8 AUT_ERROR=14
uint8 AUT_NOT_AVAILABLE=15

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

can_interfaces/CanData can_data

uint8 diff_locked
uint8 augmentation_type
float32 signal_to_noise_ratio
uint8 differential_source
uint8 accuracy
uint8 sf_active
uint8 sf2_active
uint8 rtk_active
uint8 dual_frequency
uint8 sf_activation
uint8 sf2_license
uint8 rtk_activation
uint8 at_capable
