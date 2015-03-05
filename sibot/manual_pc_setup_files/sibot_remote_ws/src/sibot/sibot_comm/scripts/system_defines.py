"""--------------------------------------------------------------------
COPYRIGHT 2014 Stanley Innovation Inc.

Software License Agreement:

The software supplied herewith by Stanley Innovation Inc. (the "Company") 
for its licensed Segway RMP Robotic Platforms is intended and supplied to you, 
the Company's customer, for use solely and exclusively with Stanley Innovation 
products. The software is owned by the Company and/or its supplier, and is 
protected under applicable copyright laws.  All rights are reserved. Any use in 
violation of the foregoing restrictions may subject the user to criminal 
sanctions under applicable laws, as well as to civil liability for the 
breach of the terms and conditions of this license. The Company may 
immediately terminate this Agreement upon your use of the software with 
any products that are not Stanley Innovation products.

The software was written using Python programming language.  Your use 
of the software is therefore subject to the terms and conditions of the 
OSI- approved open source license viewable at http://www.python.org/.  
You are solely responsible for ensuring your compliance with the Python 
open source license.

You shall indemnify, defend and hold the Company harmless from any claims, 
demands, liabilities or expenses, including reasonable attorneys fees, incurred 
by the Company as a result of any claim or proceeding against the Company 
arising out of or based upon: 

(i) The combination, operation or use of the software by you with any hardware, 
    products, programs or data not supplied or approved in writing by the Company, 
    if such claim or proceeding would have been avoided but for such combination, 
    operation or use.
 
(ii) The modification of the software by or on behalf of you 

(iii) Your use of the software.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 
 \file   system_defines.py

 \brief  This module defines the interface for the RMP

 \Platform: Linux/ROS Hydro
--------------------------------------------------------------------"""

"""
This is the maximum update rate 100Hz
"""
MIN_UPDATE_PERIOD_SEC = 0.01

"""
Flags for RMPEventHandlers class these are used to signal between threads
"""
RMP_KILL          = 1 # Sent to RMP no need to handle, but will terminate
RMP_INIT_FAILED   = 2
RMP_IS_DEAD       = 3
RMP_TX_RDY        = 4
RMP_RSP_DATA_RDY  = 5


"""------------------------------------------------------------------------
RMP Command structures
There are two types of messages the RMP will accept:
1. Motion command message: This message contains two 32-bit IEEE754 floating 
point variables. The first is the normalized velocity command (range: -1.0...1.0).
The second is the normalized yaw rate command (range: -1.0...1.0).

2. Configuration message: This message will contain two 32-bit variables. The
first is a 32-bit integer general purpose command which is the ID of the configuration 
command (see below). The second is the general purpose parameter which may be integer
or 32-bit IEEE754 floating point value depending on the command
------------------------------------------------------------------------"""

"""
Defines for the structure of commands sent to the RMP via CAN each message
consists of an ID and two 32-bit words. The CAN interface is 8-byte DLC.
The CAN data bytes are big-endian (highest byte in lowest index).
Can ID: 16-bit ID SID.
Bytes 0-3: 32-bit variable 1 (MS byte at index 0 LS byte at index 3)
Bytes 4-7: 32-bit variable 2 (MS byte at index 4 LS byte at index 7)

The format to rmp_interface.py is 
[RMP_MOTION_CMD_ID,norm_vel_cmd,norm_yaw_cmd]
[RMP_OMNI_MOTION_CMD_ID,norm_vel_cmd,norm_yaw_cmd,angle_cmd_deg]
[RMP_CFG_CMD_ID,gp_cmd,gp_param]
"""
RMP_MOTION_CMD_ID            = 0x0500
RMP_OMNI_MOTION_CMD_ID       = 0x0600
RMP_CFG_CMD_ID               = 0x0501

"""
Defines the start address for the CAN response
"""
RMP_RESPONSE_START_CAN_ID = 0x502

"""
The packet structure for USB and Ethernet. 
Bytes 0-1: 16-bit ID (see above).
Bytes 2-5: 32-bit variable 1 (content depends on message type)
Bytes 6-9: 32-bit variable 2 (content depends on message type)
Bytes 10-11: 16-bit CRC value of buffer contents
"""
RMP_USB_ETH_CAN_ID_HIGH_INDEX        = 0
RMP_USB_ETH_CAN_ID_LOW_INDEX         = 1
RMP_USB_ETH_CAN_DATA_0_INDEX         = 2
RMP_USB_ETH_CAN_DATA_1_INDEX         = 3
RMP_USB_ETH_CAN_DATA_2_INDEX         = 4
RMP_USB_ETH_CAN_DATA_3_INDEX         = 5
RMP_USB_ETH_CAN_DATA_4_INDEX         = 6
RMP_USB_ETH_CAN_DATA_5_INDEX         = 7
RMP_USB_ETH_CAN_DATA_6_INDEX         = 8
RMP_USB_ETH_CAN_DATA_7_INDEX         = 9
RMP_USB_ETH_CHECKSUM_HIGH_BYTE_INDEX = 10
RMP_USB_ETH_CHECKSUM_LOW_BYTE_INDEX  = 11

"""
Defines the total number of bytes in a USB or Ethernet packet
"""
NUM_USB_ETH_BYTES                    = 12

"""------------------------------------------------------------------------
Configuration commands
There are a number of configurable parameters on the RMP. Some are stored in
NV F-RAM memory onboard the machine. Some are runtime variables. The general
purpose command ID and the associated general purpose parameters are defined
below
------------------------------------------------------------------------"""
"""
Dictionary for all RMP configuration command ID's
"""
command_ids = dict({
                     "RMP_CMD_NONE":0,
                     "RMP_CMD_SET_MAXIMUM_VELOCITY":1,
                     "RMP_CMD_SET_MAXIMUM_ACCELERATION":2,
                     "RMP_CMD_SET_MAXIMUM_DECELERATION":3,
                     "RMP_CMD_SET_MAXIMUM_DTZ_DECEL_RATE":4,
                     "RMP_CMD_SET_COASTDOWN_ACCEL":5,
                     "RMP_CMD_SET_MAXIMUM_TURN_RATE":6,
                     "RMP_CMD_SET_MAXIMUM_TURN_ACCEL":7,
                     "RMP_CMD_SET_TIRE_DIAMETER":8,
                     "RMP_CMD_SET_WHEEL_BASE_LENGTH":9,
                     "RMP_CMD_SET_WHEEL_TRACK_WIDTH":10,
                     "RMP_CMD_SET_TRANSMISSION_RATIO":11,
                     "RMP_CMD_SET_INPUT_CONFIG_BITMAP":12,
                     "RMP_CMD_SET_ETH_IP_ADDRESS":13,
                     "RMP_CMD_SET_ETH_PORT_NUMBER":14,
                     "RMP_CMD_SET_ETH_SUBNET_MASK":15,
                     "RMP_CMD_SET_ETH_GATEWAY":16,
                     "RMP_CMD_SET_USER_FB_1_BITMAP":17,
                     "RMP_CMD_SET_USER_FB_2_BITMAP":18,
                     "RMP_CMD_SET_USER_FB_3_BITMAP":19,
                     "RMP_CMD_SET_USER_FB_4_BITMAP":20,
                     "RMP_CMD_FORCE_CONFIG_FEEDBACK_BITMAPS":30,
                     "RMP_CMD_SET_AUDIO_COMMAND":31,
                     "RMP_CMD_SET_OPERATIONAL_MODE":32,
                     "RMP_CMD_SEND_SP_FAULTLOG":33,
                     "RMP_CMD_RESET_INTEGRATORS":34,
                     "RMP_CMD_RESET_PARAMS_TO_DEFAULT":35})

int_command_params = dict({
                     "RMP_CMD_NONE":0,
                     "RMP_CMD_SET_INPUT_CONFIG_BITMAP":12,
                     "RMP_CMD_SET_ETH_IP_ADDRESS":13,
                     "RMP_CMD_SET_ETH_PORT_NUMBER":14,
                     "RMP_CMD_SET_ETH_SUBNET_MASK":15,
                     "RMP_CMD_SET_ETH_GATEWAY":16,
                     "RMP_CMD_SET_USER_FB_1_BITMAP":17,
                     "RMP_CMD_SET_USER_FB_2_BITMAP":18,
                     "RMP_CMD_SET_USER_FB_3_BITMAP":19,
                     "RMP_CMD_SET_USER_FB_4_BITMAP":20,
                     "RMP_CMD_FORCE_CONFIG_FEEDBACK_BITMAPS":30,
                     "RMP_CMD_SET_AUDIO_COMMAND":31,
                     "RMP_CMD_SET_OPERATIONAL_MODE":32,
                     "RMP_CMD_SEND_SP_FAULTLOG":33,
                     "RMP_CMD_RESET_INTEGRATORS":34,
                     "RMP_CMD_RESET_PARAMS_TO_DEFAULT":35})

"""
This command results in no action but a response from the RMP. The general
purpose parameter is ignored.
"""
RMP_CMD_NONE                                = (0)


"""------------------------------------------------------------------------
Start Variables Stored in NV F-RAM memory
------------------------------------------------------------------------"""

"""
This command updates the maximum velocity limit of the machine. The general
purpose command is a 32-bit IEEE754 floating point variable representing the
maximum velocity limit in m/s.
"""
RMP_CMD_SET_MAXIMUM_VELOCITY                = (1)

DEFAULT_MAXIMUM_VELOCITY_MPS = 2.2352
MAX_VELOCITY_MPS             = 8.047
MIN_VELOCITY_MPS             = 0.0

"""
This command updates the maximum acceleration limit of the machine. The general
purpose command is a 32-bit IEEE754 floating point variable representing the
maximum acceleration limit in m/s^2.
"""
RMP_CMD_SET_MAXIMUM_ACCELERATION            = (2)

DEFAULT_MAXIMUM_ACCELERATION_MPS2 = 3.923
MAX_ACCELERATION_MPS2             = 7.848
MIN_ACCELERATION_MPS2             = 0.0

"""
This command updates the maximum deceleration limit of the machine. The general
purpose command is a 32-bit IEEE754 floating point variable representing the
maximum deceleration limit in m/s^2.
"""
RMP_CMD_SET_MAXIMUM_DECELERATION            = (3)

DEFAULT_MAXIMUM_DECELERATION_MPS2 = 3.923 
MAX_DECELERATION_MPS2             = 7.848 
MIN_DECELERATION_MPS2             = 0.0

"""
This command updates the maximum decel to zero response deceleration limit.
The general purpose command is a 32-bit IEEE754 floating point variable 
representing the maximum DTZ response deceleration limit in m/s^2.
"""
RMP_CMD_SET_MAXIMUM_DTZ_DECEL_RATE          = (4)

DEFAULT_MAXIMUM_DTZ_DECEL_RATE_MPS2  = 3.923
MAX_DTZ_DECEL_RATE_MPS2              = 7.848
MIN_DTZ_DECEL_RATE_MPS2              = 0.0

"""
This command updates the coastdown acceleration for acceleration based input
mapping. The general purpose command is a 32-bit IEEE754 floating point variable 
representing coastdown acceleration in m/s^2.
"""
RMP_CMD_SET_COASTDOWN_ACCEL                 = (5)

DEFAULT_COASTDOWN_ACCEL_MPS2 = 0.1962
MAX_COASTDOWN_ACCEL_MPS2     = 1.961
MIN_COASTDOWN_ACCEL_MPS2     = 0.0

"""
This command updates the maximum commandable yaw rate. 
The general purpose command is a 32-bit IEEE754 floating point variable 
representing the yaw rate limit in rad/s.
""" 
RMP_CMD_SET_MAXIMUM_TURN_RATE               = (6)

DEFAULT_MAXIMUM_YAW_RATE_RPS = 3.0
MAX_YAW_RATE_RPS             = 4.5
MIN_YAW_RATE_RPS             = 0.0

"""
This command updates the maximum commandable yaw acceleration. 
The general purpose command is a 32-bit IEEE754 floating point variable 
representing yaw acceleration in rad/s^2.
""" 
RMP_CMD_SET_MAXIMUM_TURN_ACCEL              = (7)

DEFAULT_MAX_YAW_ACCEL_RPS2 = 28.274 
MAX_YAW_ACCEL_RPS2         = 28.274
MIN_YAW_ACCEL_RPS2         = 0.0

"""
This command updates the machine tire diameter used in software to calculate
velocity, position, differential wheel speed (yaw rate) and accelerations. 
The general purpose command is a 32-bit IEEE754 floating point variable 
representing tire diameter in meters.
"""
RMP_CMD_SET_TIRE_DIAMETER                   = (8)

ARTI_TIRE_DIAMETER_M    = 0.454025
I2_TIRE_DIAMETER_M      = 0.46228
X2_TIRE_DIAMETER_M      = 0.483616
OMNI_TIRE_DIAMETER_M    = 0.254
DEFAULT_TIRE_DIAMETER_M = X2_TIRE_DIAMETER_M
MAX_TIRE_DIAMETER_M     = 1.0
MIN_TIRE_DIAMETER_M     = 0.1524

"""
This command updates the machine wheel base length used in software to calculate
velocity, position, differential wheel speed (yaw rate) and accelerations. 
The general purpose command is a 32-bit IEEE754 floating point variable 
representing wheel base length in meters
"""
RMP_CMD_SET_WHEEL_BASE_LENGTH               = (9)
DEFAULT_WHEEL_BASE_LENGTH_M = 0.5842
OMNI_WHEEL_BASE_WIDTH_M     = 0.572
FLEXOMNI_WHEEL_BASE_WIDTH_M = 0.693
MAX_WHEEL_BASE_LENGTH_M     = 1.0
MIN_WHEEL_BASE_LENGTH_M     = 0.4142

"""
This command updates the machine track width (lateral distance between the tires)
used in software to calculate differential wheel speeds (yaw rate). 
The general purpose command is a 32-bit IEEE754 floating point variable 
representing track width in meters.
"""
RMP_CMD_SET_WHEEL_TRACK_WIDTH               = (10)

ARTI_WHEEL_TRACK_WIDTH_M    = 0.75565
I2_WHEEL_TRACK_WIDTH_M      = 0.569976
X2_WHEEL_TRACK_WIDTH_M      = 0.7112
OMNI_WHEEL_TRACK_WIDTH_M    = 0.693
DEFAULT_WHEEL_TRACK_WIDTH_M = X2_WHEEL_TRACK_WIDTH_M
MAX_WHEEL_TRACK_WIDTH_M     = 1.0
MIN_WHEEL_TRACK_WIDTH_M     = 0.506476

"""
This command updates the machine transmission ratio (gearbox ratio)
used in software to convert from motor speed to gearbox output speed 
The general purpose command is a 32-bit IEEE754 floating point variable 
representing transmission ratio (unitless).
"""
RMP_CMD_SET_TRANSMISSION_RATIO              = (11)

DEFAULT_TRANSMISSION_RATIO = 24.2667
MAX_TRANSMISSION_RATIO     = 200.0
MIN_TRANSMISSION_RATIO     = 1.0

"""
This command updates the machine configuration bitmap. This bitmap sets
the controller input mapping for vel/yaw controllers and whether the machine
should use audio.
The general purpose command is a 32-bit integer variable with bits representing
each variable.
"""
RMP_CMD_SET_INPUT_CONFIG_BITMAP             = (12)

YAW_ALAT_SCALE_MAPPING       = 0
YAW_ALAT_LIMIT_MAPPING       = 1

VELOCITY_BASED_MAPPING       = 0
ACCELERATION_BASED_MAPPING   = 1

ALLOW_MACHINE_AUDIO          = 0
SILENCE_MACHINE_AUDIO        = 1

DISABLE_AC_PRESENT_CSI       = 1
ENABLE_AC_PRESENT_CSI        = 0

BALANCE_MODE_DISABLED        = 0
BALANCE_MODE_ENABLED         = 1

BALANCE_GAINS_DEFAULT        = (0x00000000)
BALANCE_GAINS_LIGHT          = (0x00000001)
BALANCE_GAINS_TALL           = (0x00000002)
BALANCE_GAINS_HEAVY          = (0x00000004)
BALANCE_GAINS_CUSTOM         = (0x00000008)
VALID_BALANCE_GAINS_MASK     = (0x0000000F)

VEL_MAPPING_NO_FILTER        = (0x00000000)
VEL_MAPPING_4HZ_FILTER       = (0x00000001)
VEL_MAPPING_1HZ_FILTER       = (0x00000002)
VEL_MAPPING_05HZ_FILTER      = (0x00000004)
VEL_MAPPING_02HZ_FILTER      = (0x00000008)
VALID_BALANCE_GAINS_MASK     = (0x0000000F)

YAW_INPUT_MAPPING_SHIFT      = 0
VEL_INPUT_MAPPING_SHIFT      = 1
AUDIO_SILENCE_REQUEST_SHIFT  = 2
DISABLE_AC_PRESENT_CSI_SHIFT = 3
BALANCE_GAIN_SCHEDULE_SHIFT  = 4
BALANCE_MODE_LOCKOUT_SHIFT   = 8
VEL_MAPPING_FILTER_SHIFT     = 9


DEFAULT_CONFIG_BITMAP = ((YAW_ALAT_LIMIT_MAPPING << YAW_INPUT_MAPPING_SHIFT) |
                         (VELOCITY_BASED_MAPPING << VEL_INPUT_MAPPING_SHIFT) |
                         (ALLOW_MACHINE_AUDIO << AUDIO_SILENCE_REQUEST_SHIFT)|
                         (DISABLE_AC_PRESENT_CSI << DISABLE_AC_PRESENT_CSI_SHIFT)|
                         (BALANCE_GAINS_DEFAULT << BALANCE_GAIN_SCHEDULE_SHIFT)|
                         (BALANCE_MODE_DISABLED << BALANCE_MODE_LOCKOUT_SHIFT) |
                         (VEL_MAPPING_NO_FILTER << VEL_MAPPING_FILTER_SHIFT))

RMP220_BALANCING_CONFIG_BITMAP = ((YAW_ALAT_LIMIT_MAPPING << YAW_INPUT_MAPPING_SHIFT) |
                                  (VELOCITY_BASED_MAPPING << VEL_INPUT_MAPPING_SHIFT) |
                                  (ALLOW_MACHINE_AUDIO << AUDIO_SILENCE_REQUEST_SHIFT)|
                                  (DISABLE_AC_PRESENT_CSI << DISABLE_AC_PRESENT_CSI_SHIFT)|
                                  (BALANCE_GAINS_DEFAULT << BALANCE_GAIN_SCHEDULE_SHIFT)|
                                  (BALANCE_MODE_ENABLED << BALANCE_MODE_LOCKOUT_SHIFT) |
                                  (VEL_MAPPING_NO_FILTER << VEL_MAPPING_05HZ_FILTER))
"""
This command updates the machine ethernet IP address.
The general purpose command is a 32-bit integer variable representing a 
dotted quad. 
The conversion is:
integer = (first octet * 16777216) + (second octet * 65536) + (third octet * 256) + (fourth octet)
Bounds for this item are valid IP addresses
"""
RMP_CMD_SET_ETH_IP_ADDRESS                  = (13)

DEFAULT_IP_ADDRESS  = '192.168.0.40'

"""
This command updates the machine ethernet port number.
The general purpose command is a 32-bit integer variable
Bounds for this item are valid ethernet ports
"""
RMP_CMD_SET_ETH_PORT_NUMBER                 = (14)

DEFAULT_PORT_NUMBER = 8080

"""
This command updates the machine ethernet IP subnet mask.
The general purpose command is a 32-bit integer variable representing a 
dotted quad. 
The conversion is:
integer = (first octet * 16777216) + (second octet * 65536) + (third octet * 256) + (fourth octet)
Bounds for this item are valid IP subnet masks
"""
RMP_CMD_SET_ETH_SUBNET_MASK                 = (15)

DEFAULT_SUBNET_MASK = '255.255.255.0'

"""
This command updates the machine ethernet IP gateway.
The general purpose command is a 32-bit integer variable representing a 
dotted quad. 
The conversion is:
integer = (first octet * 16777216) + (second octet * 65536) + (third octet * 256) + (fourth octet)
Bounds for this item are valid IP gateway
"""
RMP_CMD_SET_ETH_GATEWAY                     = (16)

DEFAULT_GATEWAY     = '192.168.0.1'

"""
This command updates user feedback bitmap 1. It is used to
select feedback from the dictionary at the top of the file.
The general purpose parameter is a 32 bit integer
"""
RMP_CMD_SET_USER_FB_1_BITMAP                = (17)
DEFAULT_USER_FB_1_BITMAP    = 0xFFFFFFFF

"""
This command updates user feedback bitmap 2. It is used to
select feedback from the dictionary at the top of the file.
The general purpose parameter is a 32 bit integer
"""
RMP_CMD_SET_USER_FB_2_BITMAP                = (18)
DEFAULT_USER_FB_2_BITMAP    = 0xFFFFFFFF
VALID_USER_FB_2_BITMAP_MASK = 0xFFFFFFFF

"""
This command updates user feedback bitmap 3. It is used to
select feedback from the dictionary at the top of the file.
The general purpose parameter is a 32 bit integer
"""
RMP_CMD_SET_USER_FB_3_BITMAP                = (19)
DEFAULT_USER_FB_3_BITMAP    = 0xFFFFFFFF

"""
This command updates user feedback bitmap 3. It is used to
select feedback from the dictionary at the top of the file.
The general purpose parameter is a 32 bit integer
"""
RMP_CMD_SET_USER_FB_4_BITMAP                = (20)
DEFAULT_USER_FB_4_BITMAP    = 0x00000000

"""
This commands sets the maximum lateral accel. It defines the yaw rate limit for a
given velocity. The lower the value the more yaw rate will be limited at speed.
"""
RMP_CMD_SET_MAXIMUM_LATERAL_ACCEL           = (21)
DEFAULT_MAXIMUM_LATERAL_ACCEL_MPS2          = 4.905
MAX_LATERAL_ACCEL_MPS2                      = 9.81
MIN_LATERAL_ACCEL_MPS2                      = 0.981


"""
When the RMP_CMD_FORCE_CONFIG_FEEDBACK_BITMAPS command is set this is the 
size of the response array. The index of the items matches the parameter number -1
for the items defined above. +1 is for the CRC
"""
NUMBER_OF_NVM_CONFIG_PARAMS = 21 #should match last cmd param id above
FORCED_CONFIG_FEEDBACK_ITEMS  = NUMBER_OF_NVM_CONFIG_PARAMS + 1
 

"""------------------------------------------------------------------------
End Variables Stored in NV F-RAM memory
------------------------------------------------------------------------"""

"""
This command forces the feedback to be the configurable items stored in NV memory above.
It is used when verifying that paremeters have been successfully set or just general verification
at startup. The general purpose parameter is 1 to force the feedback to contain configurable items
and 0 to stop forcing the feedback.
"""
RMP_CMD_FORCE_CONFIG_FEEDBACK_BITMAPS       = (30)

"""
This command requests audio songs on the machine. The general purpose parameter 
is a 32 bit integer representing the song request ID. Some songs are presistant (ie they must be
manually cleared by sending the NO_SONG after set). Most are momentary.
""" 
RMP_CMD_SET_AUDIO_COMMAND                   = (31)

MOTOR_AUDIO_PLAY_NO_SONG                  = (0)
MOTOR_AUDIO_PLAY_POWER_ON_SONG            = (1)
MOTOR_AUDIO_PLAY_POWER_OFF_SONG           = (2)
MOTOR_AUDIO_PLAY_ALARM_SONG               = (3)
MOTOR_AUDIO_PLAY_MODE_UP_SONG             = (4)
MOTOR_AUDIO_PLAY_MODE_DOWN_SONG           = (5)
MOTOR_AUDIO_PLAY_ENTER_ALARM_SONG         = (6)
MOTOR_AUDIO_PLAY_EXIT_ALARM_SONG          = (7)
MOTOR_AUDIO_PLAY_FINAL_SHUTDOWN_SONG      = (8)
MOTOR_AUDIO_PLAY_CORRECT_ISSUE            = (9)
MOTOR_AUDIO_PLAY_ISSUE_CORRECTED          = (10)
MOTOR_AUDIO_PLAY_CORRECT_ISSUE_REPEATING  = (11)
MOTOR_AUDIO_PLAY_BEGINNER_ACK             = (12)
MOTOR_AUDIO_PLAY_EXPERT_ACK               = (13)
MOTOR_AUDIO_ENTER_FOLLOW                  = (14)
MOTOR_AUDIO_TEST_SWEEP                    = (15)
MOTOR_AUDIO_SIMULATE_MOTOR_NOISE          = (16)

"""
This command updates the operational mode request on the machine. The general purpose parameter 
is a 32 bit integer representing the mode request ID.
"""
RMP_CMD_SET_OPERATIONAL_MODE                = (32)

DISABLE_REQUEST   = 1
POWERDOWN_REQUEST = 2
DTZ_REQUEST       = 3
STANDBY_REQUEST   = 4
TRACTOR_REQUEST   = 5
BALANCE_REQUEST   = 6

"""
This command requests the faultlog from the machine. The general purpose parameter 
is 1 indicating this is a new request, or 0 indicating it is a subsequent request.
The entire faultlog requires 6 packets, the first request should have the general 
purpose command set to 1 the next 6 should have the general purpose command set to 0.
"""
RMP_CMD_SEND_SP_FAULTLOG                    = (33)

FAULTLOG_INITIAL_REQUEST = 1
FAULTLOG_NORMAL_REQUEST  = 0

"""
Define the number of faultlog packets and the size (in 32-bit words) of each packet.
"""
FAULTLOG_NUM_OF_PACKETS = 6
FAULTLOG_PACKET_NUM_OF_WORDS = 53

"""
This command resets the position data on the machine. The general purpose command
is a bitmap of which integrators are to be reset.
""" 
RMP_CMD_RESET_INTEGRATORS                  = (34)

RESET_LINEAR_POSITION      = 0x00000001
RESET_RIGHT_FRONT_POSITION = 0x00000002
RESET_LEFT_FRONT_POSITION  = 0x00000004
RESET_RIGHT_REAR_POSITION  = 0x00000008
RESET_LEFT_REAR_POSITION   = 0x00000010
RESET_ALL_POSITION_DATA    = 0x0000001F

"""
This command resets all configurable parameters to their default values. 
The general purpose parameter is ignored for this request.
""" 
RMP_CMD_RESET_PARAMS_TO_DEFAULT            = (35)


"""------------------------------------------------------------------------
RMP Fault definitions
This section is used to define the decoding of fault status words sent 
by the RMP. The meaning of specific faults can be found in the interface
guide.
------------------------------------------------------------------------"""
NO_FAULT                                    = 0x00000000
ALL_FAULTS                                  = 0xFFFFFFFF

"""
Transient faults: These faults are not latching and can be asserted and then
cleared during runtime. There are currently no transient faults for the RMP
"""
transient_fault_decode = dict({
    0x00000000: ""})

"""
Critical faults: These faults are latching.
"""
critical_fault_decode = dict({
    0x00000000: "",                          
    0x00000001:"CRITICAL_FAULT_INIT",
    0x00000002:"CRITICAL_FAULT_INIT_UIP_COMM",
    0x00000004:"CRITICAL_FAULT_INIT_PROPULSION",
    0x00000008:"CRITICAL_FAULT_INIT_TIMEOUT",
    0x00000010:"CRITICAL_FAULT_FORW_SPEED_LIMITER_HAZARD",
    0x00000020:"CRITICAL_FAULT_AFT_SPEED_LIMITER_HAZARD",
    0x00000040:"CRITICAL_FAULT_CHECK_STARTUP",
    0x00000080:"CRITICAL_FAULT_APP_VELOCITY_CTL_FAILED",
    0x00000100:"CRITICAL_FAULT_APP_POSITION_CTL_FAILED",
    0x00000200:"CRITICAL_FAULT_ABB_SHUTDOWN",
    0x00000400:"CRITICAL_FAULT_AP_MODE_TRANS_TIMEOUT",
    0x00000800:"CRITICAL_FAULT_PITCH_ANGLE_EXCEEDED",
    0x00001000:"CRITICAL_FAULT_ROLL_ANGLE_EXCEEDED",
    0x00002000:"CRITICAL_FAULT_BSB_INIT_FAILED",
    0x00004000:"CRITICAL_FAULT_BSB_COMM_FAILED",
    0x00008000:"CRITICAL_FAULT_BSB_LOST_POWER",
    0x00010000:"CRITICAL_FAULT_BSB_HW_FAULT"})

"""
Communication faults: These faults are latching.
"""
comm_fault_decode = dict({
    0x00000000: "",
    0x00000001:"COMM_FAULT_UIP_MISSING_UIP_DATA",
    0x00000002:"COMM_FAULT_UIP_UNKNOWN_MESSAGE_RECEIVED",
    0x00000004:"COMM_FAULT_UIP_BAD_CHECKSUM",
    0x00000008:"COMM_FAULT_UIP_TRANSMIT",
    0x00000010:"COMM_FAULT_UI_BAD_MOTION_CMD",
    0x00000020:"COMM_FAULT_UI_UNKOWN_CMD",
    0x00000040:"COMM_FAULT_UI_BAD_PACKET_CHECKSUM"})


"""
MCU faults: These faults are latching.
"""
mcu_fault_decode = dict({
    0x00000000: "",                     
    0x00000001:"MCU_FAULT_MCU_0_IS_DEGRADED",
    0x00000002:"MCU_FAULT_MCU_0_IS_FAILED",
    0x00000004:"MCU_FAULT_MCU_0_REQUESTS_REDUCED_PERFORMANCE",
    0x00000008:"MCU_FAULT_MCU_0_REQUESTS_ZERO_SPEED",
    0x00000010:"MCU_FAULT_MCU_1_IS_DEGRADED",
    0x00000020:"MCU_FAULT_MCU_1_IS_FAILED",
    0x00000040:"MCU_FAULT_MCU_1_REQUESTS_REDUCED_PERFORMANCE",
    0x00000080:"MCU_FAULT_MCU_1_REQUESTS_ZERO_SPEED",
    0x00000100:"MCU_FAULT_MCU_2_IS_DEGRADED",
    0x00000200:"MCU_FAULT_MCU_2_IS_FAILED",
    0x00000400:"MCU_FAULT_MCU_2_REQUESTS_REDUCED_PERFORMANCE",
    0x00000800:"MCU_FAULT_MCU_2_REQUESTS_ZERO_SPEED",
    0x00001000:"MCU_FAULT_MCU_3_IS_DEGRADED",
    0x00002000:"MCU_FAULT_MCU_3_IS_FAILED",
    0x00004000:"MCU_FAULT_MCU_3_REQUESTS_REDUCED_PERFORMANCE",
    0x00008000:"MCU_FAULT_MCU_3_REQUESTS_ZERO_SPEED",
    0x00010000:"MCU_FAULT_MISSING_MCU_0_DATA",
    0x00020000:"MCU_FAULT_MISSING_MCU_1_DATA",
    0x00040000:"MCU_FAULT_MISSING_MCU_2_DATA",
    0x00080000:"MCU_FAULT_MISSING_MCU_3_DATA",
    0x00100000:"MCU_FAULT_UNKNOWN_MESSAGE_RECEIVED"})

"""
Define a mask to indicate that the CCU has detected the fault and not the MCU
"""
CCU_DETECTED_MCU_FAULT_MASK = 0x001F0000

"""
Sensor faults: These faults are latching.
"""
sensor_fault_decode = dict({
    0x00000000: "",                        
    0x00000001:"SENSOR_FAULT_2P5V_VREF_RANGE_FAULT",
    0x00000002:"SENSOR_FAULT_7P2V_VBAT_RANGE_FAULT",
    0x00000004:"SENSOR_FAULT_7P2V_VBAT_WARNING",
    0x00000008:"SENSOR_FAULT_7P2V_BATT_INBALANCE_FAULT",
    0x00000010:"SENSOR_FAULT_7P2V_BATT_TEMPERATURE_FAULT",
    0x00000020:"SENSOR_FAULT_DIGITAL_INPUT",
    0x00000040:"SENSOR_FAULT_RANGE",
    0x00000080:"SENSOR_FAULT_DEFAULT",
    0x00000100:"SENSOR_FAULT_5V_MONITOR_RANGE_FAULT",
    0x00000200:"SENSOR_FAULT_12V_MONITOR_RANGE_FAULT"})
 
"""
BSA faults: These faults are latching.
"""
bsa_fault_decode = dict({
    0x00000000: "",                     
    0x00000001:"BSA_FAULT_SIDE_A_MISSING_BSA_DATA",
    0x00000002:"BSA_FAULT_SIDE_B_MISSING_BSA_DATA",
    0x00000004:"BSA_FAULT_UNKNOWN_MESSAGE_RECEIVED",
    0x00000008:"BSA_FAULT_TRANSMIT_A_FAILED",
    0x00000010:"BSA_FAULT_TRANSMIT_B_FAILED",
    0x00000020:"BSA_FAULT_DEFAULT",
    0x00000040:"BSA_FAULT_SIDE_A_RATE_SENSOR_SATURATED",
    0x00000080:"BSA_FAULT_SIDE_B_RATE_SENSOR_SATURATED",
    0x00000100:"BSA_FAULT_SIDE_A_TILT_SENSOR_SATURATED",
    0x00000200:"BSA_FAULT_SIDE_B_TILT_SENSOR_SATURATED",
    0x00000400:"PSE_FAULT_COMPARISON"})

"""
Architecture faults: These faults are latching.
"""
arch_fault_decode = dict({
    0x00000000: "",                      
    0x00000001:"ARCHITECT_FAULT_SPI_RECEIVE",
    0x00000002:"ARCHITECT_FAULT_SPI_TRANSMIT",
    0x00000004:"ARCHITECT_FAULT_SPI_RECEIVE_OVERRUN",
    0x00000008:"ARCHITECT_FAULT_SPI_RX_BUFFER_OVERRUN",
    0x00000010:"ARCHITECT_FAULT_COMMANDED_SAFETY_SHUTDOWN",
    0x00000020:"ARCHITECT_FAULT_COMMANDED_DISABLE",
    0x00000040:"ARCHITECT_FAULT_KILL_SWITCH_ACTIVE",
    0x00000080:"ARCHITECT_FAULT_FRAM_CONFIG_INIT_FAILED",
    0x00000100:"ARCHITECT_FAULT_FRAM_CONFIG_SET_FAILED",
    0x00000200:"ARCHITECT_FAULT_BAD_MODEL_IDENTIFIER",
    0x00000400:"ARCHITECT_FAULT_BAD_CCU_HW_REV",
    0x00000800:"ARCHITECT_FAULT_DECEL_SWITCH_ACTIVE"})

"""
Internal faults: These faults are latching.
"""
internal_fault_decode = dict({
    0x00000000: "",                          
    0x00000001:"INTERNAL_FAULT_HIT_DEFAULT_CONDITION",
    0x00000002:"INTERNAL_FAULT_HIT_SPECIAL_CASE"})

"""
MCU specific faults: These faults are detected locally by the MCU
"""
mcu_specific_fault_decode = dict({
    0x00000000: "",                              
    0x00000001:"MCU_TRANS_BATTERY_TEMP_WARNING",
    0x00000002:"MCU_TRANS_BATTERY_COLD_REGEN",
    0x00000004:"MCU_UNKNOWN",
    0x00000008:"MCU_UNKNOWN",
    0x00000010:"MCU_TRANS_LOW_BATTERY",
    0x00000020:"MCU_TRANS_BATT_OVERVOLTAGE",
    0x00000040:"MCU_CRITICAL_BATT_OVERVOLTAGE",
    0x00000080:"MCU_CRITICAL_EMPTY_BATTERY",
    0x00000100:"MCU_CRITICAL_BATTERY_TEMP",
    0x00000200:"MCU_COMM_CU_BCU_LINK_DOWN",
    0x00000400:"MCU_COMM_INITIALIZATION_FAILED",
    0x00000800:"MCU_COMM_FAILED_CAL_EEPROM",
    0x00001000:"MCU_POWER_SUPPLY_TRANSIENT_FAULT",
    0x00002000:"MCU_POWER_SUPPLY_12V_FAULT",
    0x00004000:"MCU_POWER_SUPPLY_5V_FAULT",
    0x00008000:"MCU_POWER_SUPPLY_3V_FAULT",
    0x00010000:"MCU_JUNCTION_TEMP_FAULT",
    0x00020000:"MCU_MOTOR_WINDING_TEMP_FAULT",
    0x00040000:"MCU_MOTOR_DRIVE_FAULT",
    0x00080000:"MCU_MOTOR_DRIVE_HALL_FAULT",
    0x00100000:"MCU_MOTOR_DRIVE_AMP_FAULT",
    0x00200000:"MCU_MOTOR_DRIVE_AMP_ENABLE_FAULT",
    0x00400000:"MCU_MOTOR_DRIVE_AMP_OVERCURRENT_FAULT",
    0x00800000:"MCU_MOTOR_DRIVE_VOLTAGE_FEEDBACK_FAULT",
    0x01000000:"MCU_FRAME_FAULT",
    0x02000000:"MCU_BATTERY_FAULT",
    0x08000000:"MCU_MOTOR_STUCK_RELAY_FAULT",
    0x10000000:"MCU_ACTUATOR_POWER_CONSISTENCY_FAULT",
    0x20000000:"MCU_ACTUATOR_HALT_PROCESSOR_FAULT",
    0x40000000:"MCU_ACTUATOR_DEGRADED_FAULT"})

"""
All the fault groups are packed into four 32-bit fault status words. The following
defines how they are packed into the words
"""

"""
Fault status word 0
"""
FSW_ARCH_FAULTS_INDEX       = 0
FSW_ARCH_FAULTS_SHIFT       = 0
FSW_ARCH_FAULTS_MASK        = 0x00000FFF
FSW_CRITICAL_FAULTS_INDEX   = 0
FSW_CRITICAL_FAULTS_SHIFT   = 12
FSW_CRITICAL_FAULTS_MASK    = 0xFFFFF000
"""
Fault status word 1
"""
FSW_COMM_FAULTS_INDEX       = 1
FSW_COMM_FAULTS_SHIFT       = 0
FSW_COMM_FAULTS_MASK        = 0x0000FFFF
FSW_INTERNAL_FAULTS_INDEX   = 1
FSW_INTERNAL_FAULTS_SHIFT   = 16
FSW_INTERNAL_FAULTS_MASK    = 0x000F0000
"""
Fault status word 2
"""
FSW_SENSORS_FAULTS_INDEX    = 2
FSW_SENSORS_FAULTS_SHIFT    = 0
FSW_SENSORS_FAULTS_MASK     = 0x0000FFFF
FSW_BSA_FAULTS_INDEX        = 2
FSW_BSA_FAULTS_SHIFT        = 16
FSW_BSA_FAULTS_MASK         = 0xFFFF0000
"""
Fault status word 3
"""
FSW_MCU_FAULTS_INDEX        = 3
FSW_MCU_FAULTS_SHIFT        = 0
FSW_MCU_FAULTS_MASK         = 0xFFFFFFFF

"""
Fault group index definitions
"""
FAULTGROUP_TRANSIENT    = 0
FAULTGROUP_CRITICAL     = 1
FAULTGROUP_COMM         = 2
FAULTGROUP_SENSORS      = 3
FAULTGROUP_BSA          = 4
FAULTGROUP_MCU          = 5
FAULTGROUP_ARCHITECTURE = 6  
FAULTGROUP_INTERNAL     = 7
NUM_OF_FAULTGROUPS      = 8


