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
 
 \file   sibot_config_params.py

 \brief  Defines the user configurable parameters for loading at
         runtime

 \Platform: Linux/ROS Hydro
--------------------------------------------------------------------"""
from system_defines import *
from utils import *
import rospy


SIBOT_FB_1_BITMAP = 0xFFFFFFFF
SIBOT_FB_2_BITMAP = 0xFFFFFFFF
SIBOT_FB_3_BITMAP = 0x007FFFFF
SIBOT_FB_4_BITMAP = 0x00000000

"""
Define the configuration parameters for all the platforms
"""
sibot_cfg_params = [convert_float_to_u32(rospy.get_param('/sibot_configs/vel_limit_mps')),
                    convert_float_to_u32(rospy.get_param('/sibot_configs/accel_limit_mps2')),
                    convert_float_to_u32(rospy.get_param('/sibot_configs/decel_limit_mps2')),
                    convert_float_to_u32(rospy.get_param('/sibot_configs/dtz_decel_limit_mps2')),
                    convert_float_to_u32(0.1962), #costdown accel mps2
                    convert_float_to_u32(rospy.get_param('/sibot_configs/yaw_rate_limit_rps')),
                    convert_float_to_u32(rospy.get_param('/sibot_configs/yaw_accel_limit_rps2')),
                    convert_float_to_u32(I2_TIRE_DIAMETER_M),
                    convert_float_to_u32(DEFAULT_WHEEL_BASE_LENGTH_M),
                    convert_float_to_u32(I2_WHEEL_TRACK_WIDTH_M),
                    convert_float_to_u32(DEFAULT_TRANSMISSION_RATIO),
                    DEFAULT_CONFIG_BITMAP,
                    dottedQuadToNum('192.168.1.13'),
                    DEFAULT_PORT_NUMBER,
                    dottedQuadToNum(DEFAULT_SUBNET_MASK),
                    dottedQuadToNum('192.168.1.1'),
                    SIBOT_FB_1_BITMAP,
                    SIBOT_FB_2_BITMAP,
                    SIBOT_FB_3_BITMAP,
                    SIBOT_FB_4_BITMAP,
                    convert_float_to_u32(DEFAULT_MAXIMUM_LATERAL_ACCEL_MPS2)]

"""------------------------------------------------------------------------
RMP Feedback dictionaries:
There are three 32-bit user feedback bitmaps that define the content of the 
RMP response message. Each bit in a bitmap corresponds to a feedback variable.
If the bit is set in the configurable parameter the item is included in the 
response packet at the index defined by its order in the bitmaps. This means
that if the first bit of feedback bitmap one is set it will be the variable 
at the first index in the reponse if the last bit of feedback bitmap 3 is set
it will be the variable at the last index (before the CRC) of the response.

Response packets are big endian and each variable consists of 32 bytes. The variable
representation in human readable form is defined by the masks below.    
------------------------------------------------------------------------"""


"""
Dictionary for user feedback bitmap 1 and associated masks for
representing data
"""
RMP_FORCED_FEEDBACK_1_MASK         = 0x00000000
RMP_FLOATING_POINT_FEEDBACK_1_MASK = 0xFFFF7F20
RMP_DOUBLE_FEEDBACK_1_MASK         = 0x00000000
RMP_HEX_FEEDBACK_1_MASK            = 0x0000801F
RMP_IP_FEEDBACK_1_MASK             = 0x00000000

feedback_1_bitmap_menu_items = dict({
                            (1<<0):"fault_status_word_1",
                            (1<<1):"fault_status_word_2",
                            (1<<2):"fault_status_word_3",
                            (1<<3):"fault_status_word_4",
                            (1<<4):"mcu_1_fault_status",
                            (1<<5):"operational_time",
                            (1<<6):"operational_state",
                            (1<<7):"dynamic_response",
                            (1<<8):"mcu_1_battery_soc",
                            (1<<9):"mcu_1_battery_temp_degC",
                            (1<<10):"min_propulsion_batt_soc",
                            (1<<11):"aux_batt_soc",
                            (1<<12):"aux_batt_voltage_V",
                            (1<<13):"aux_batt_current_A",
                            (1<<14):"aux_batt_temp_degC",
                            (1<<15):"abb_status",                                                    
                            (1<<16):"right_motor_current_A0pk",
                            (1<<17):"left_motor_current_A0pk",
                            (1<<18):"max_motor_current_A0pk",
                            (1<<19):"right_motor_current_limit_A0pk",
                            (1<<20):"left_motor_current_limit_A0pk",
                            (1<<21):"min_motor_current_limit_A0pk",
                            (1<<22):"mcu_1_inst_power_W",
                            (1<<23):"mcu_1_total_energy_Wh",                       
                            (1<<24):"vel_limit_mps",                            
                            (1<<25):"yaw_rate_limit_rps",
                            (1<<26):"linear_accel_mps2",
                            (1<<27):"linear_vel_mps",
                            (1<<28):"differential_wheel_vel_rps",
                            (1<<29):"right_wheel_vel_mps",
                            (1<<30):"left_wheel_vel_mps",
                            (1<<31):"right_wheel_pos_m"})

"""
Dictionary for user feedback bitmap 2 and associated masks for
representing data
"""
RMP_FORCED_FEEDBACK_2_MASK         = 0x0FFFFF80
RMP_FLOATING_POINT_FEEDBACK_2_MASK = 0xF803FFFF
RMP_DOUBLE_FEEDBACK_2_MASK         = 0x00000000
RMP_HEX_FEEDBACK_2_MASK            = 0x07840000
RMP_IP_FEEDBACK_2_MASK             = 0x00680000
                         
feedback_2_bitmap_menu_items = dict({
                            (1<<0):"left_wheel_pos_m",
                            (1<<1):"linear_pos_m",
                            (1<<2):"vel_target_mps",
                            (1<<3):"yaw_rate_target_rps",
                            (1<<4):"odom_est_x_m",
                            (1<<5):"odom_est_y_m",
                            (1<<6):"odom_est_yaw_angle_rad",
                            (1<<7):"fram_vel_limit_mps",
                            (1<<8):"fram_accel_limit_mps2",
                            (1<<9):"fram_decel_limit_mps2",
                            (1<<10):"fram_dtz_decel_limit_mps2",
                            (1<<11):"fram_coastdown_decel_mps2",
                            (1<<12):"fram_yaw_rate_limit_rps",
                            (1<<13):"fram_yaw_accel_limit_rps2",
                            (1<<14):"fram_tire_diameter_m",
                            (1<<15):"fram_wheelbase_length_m",
                            (1<<16):"fram_wheel_track_width_m",
                            (1<<17):"fram_transmission_ratio",
                            (1<<18):"fram_config_bitmap",
                            (1<<19):"fram_eth_ip_address",
                            (1<<20):"fram_eth_port_number",
                            (1<<21):"fram_eth_subnet_mask",
                            (1<<22):"fram_eth_gateway",
                            (1<<23):"fram_user_feedback_bitmap_1",
                            (1<<24):"fram_user_feedback_bitmap_2",
                            (1<<25):"fram_user_feedback_bitmap_3",
                            (1<<26):"fram_user_feedback_bitmap_4",
                            (1<<27):"fram_lateral_accel_limit_mps2",
                            (1<<28):"imu_x_accel_g",
                            (1<<29):"imu_y_accel_g",
                            (1<<30):"imu_z_accel_g",
                            (1<<31):"imu_x_rate_rps"})

"""
Dictionary for user feedback bitmap 3 and associated masks for
representing data
"""
RMP_FORCED_FEEDBACK_3_MASK         = 0x00000000
RMP_FLOATING_POINT_FEEDBACK_3_MASK = 0x000601FF
RMP_DOUBLE_FEEDBACK_3_MASK         = 0x0001FE00
RMP_HEX_FEEDBACK_3_MASK            = 0x01F80002
RMP_IP_FEEDBACK_3_MASK             = 0x00000000

feedback_3_bitmap_menu_items = dict({
                            (1<<0):"imu_y_rate_rps",
                            (1<<1):"imu_z_rate_rps",
                            (1<<2):"imu_x_mag_G",
                            (1<<3):"imu_y_mag_G",
                            (1<<4):"imu_z_mag_G",
                            (1<<5):"imu_roll_deg",
                            (1<<6):"imu_pitch_deg",
                            (1<<7):"imu_yaw_deg",
                            (1<<8):"imu_timestamp_sec",
                            (1<<9):"gps_latitude_deg",
                            (1<<10):"gps_latitude_deg",
                            (1<<11):"gps_longitude_deg",
                            (1<<12):"gps_longitude_deg",
                            (1<<13):"gps_el_height_m",
                            (1<<14):"gps_el_height_m",
                            (1<<15):"gps_msl_height_m",
                            (1<<16):"gps_msl_height_m",
                            (1<<17):"gps_horizontal_stdev",
                            (1<<18):"gps_vertical_stdev",
                            (1<<19):"gps_valid_llh_flag",
                            (1<<20):"gps_packed_fix_info",
                            (1<<21):"gps_packed_hw_status",
                            (1<<22):"imu_toolkit_status",
                            (1<<23):"imu_toolkit_errors",
                            (1<<24):"packed_build_ids"})

"""
Dictionary for user feedback bitmap 4 and associated masks for
representing data
"""
RMP_FORCED_FEEDBACK_4_MASK         = 0x00000000
RMP_FLOATING_POINT_FEEDBACK_4_MASK = 0x00000000
RMP_DOUBLE_FEEDBACK_4_MASK         = 0x00000000
RMP_HEX_FEEDBACK_4_MASK            = 0x00000000
RMP_IP_FEEDBACK_4_MASK             = 0x00000000

feedback_4_bitmap_menu_items = dict()
