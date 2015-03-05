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
 
 \file   rmp_data_classes.py

 \brief  a collection of RMP data classes

 \Platform: Linux/ROS Hydro
--------------------------------------------------------------------"""
from utils import *
from sibot_comm.msg import *
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField,NavSatFix,NavSatStatus,JointState
from robot_localization.srv import SetPose
import rospy, tf, math

class SIBOT_Status:
    def __init__(self):
        self._MsgData = sibotStatus()
        self._MsgPub = rospy.Publisher('/sibot/feedback/status', sibotStatus)
        self._MsgData.header.frame_id = ''
        self._seq = 0
        
    def parse(self,data):

        self._MsgData.header.stamp = rospy.Time.now()
        self._MsgData.header.seq = self._seq
        
        temp = [data[0],data[1],data[2],data[3]]
        self._MsgData.fault_status_words = temp
        temp = [0,data[4],0,0]
        self._MsgData.mcu_fault_status = temp
        self._MsgData.operational_time = convert_u32_to_float(data[5])
        self._MsgData.operational_state = data[6]
        self._MsgData.dynamic_response = data[7]
        
        self._MsgPub.publish(self._MsgData)
        self._seq += 1        

class SIBOT_Batteries:
    def __init__(self):
        self._MsgData = sibotBattery()
        self._MsgPub = rospy.Publisher('/sibot/feedback/battery', sibotBattery)
        self._MsgData.header.frame_id = ''
        self._seq = 0
        
    def parse(self,data):
        self.mcu_1_battery_soc = convert_u32_to_float(data[0])
        self.mcu_1_battery_temp_degC = convert_u32_to_float(data[1])
        self.min_propulsion_batt_soc = convert_u32_to_float(data[2])
        self.aux_batt_soc = convert_u32_to_float(data[3])
        self.aux_batt_voltage_V = convert_u32_to_float(data[4])
        self.aux_batt_current_A = convert_u32_to_float(data[5])
        self.aux_batt_temp_degC = convert_u32_to_float(data[6])
        self.abb_status = data[7]
        
        self._MsgData.header.stamp = rospy.Time.now()
        self._MsgData.header.seq = self._seq
        
        temp = [0.0,convert_u32_to_float(data[0]),0.0,0.0]
        self._MsgData.mcu_battery_soc = temp
        temp = [0.0,convert_u32_to_float(data[1]),0.0,0.0]
        self._MsgData.mcu_battery_temp_degC = temp
        self._MsgData.min_propulsion_batt_soc = convert_u32_to_float(data[2])
        self._MsgData.aux_batt_soc = convert_u32_to_float(data[3])
        self._MsgData.aux_batt_voltage_V = convert_u32_to_float(data[4])
        self._MsgData.aux_batt_current_A = convert_u32_to_float(data[5])
        self._MsgData.aux_batt_temp_degC = convert_u32_to_float(data[6])
        self._MsgData.abb_status = data[7]
        
        self._MsgPub.publish(self._MsgData)
        self._seq += 1
        
class SIBOT_Propulsion:
    def __init__(self):
        self._MsgData = sibotPropulsion()
        self._MsgPub = rospy.Publisher('/sibot/feedback/propulsion', sibotPropulsion)
        self._MsgData.header.frame_id = ''
        self._seq = 0
        
    def parse(self,data):
        self._MsgData.header.stamp = rospy.Time.now()
        self._MsgData.header.seq = self._seq        
        
        self._MsgData.right_motor_current_A0pk = convert_u32_to_float(data[0])
        self._MsgData.left_motor_current_A0pk = convert_u32_to_float(data[1])
        self._MsgData.max_motor_current_A0pk = convert_u32_to_float(data[2])
        self._MsgData.right_motor_current_limit_A0pk = convert_u32_to_float(data[3])
        self._MsgData.left_motor_current_limit_A0pk = convert_u32_to_float(data[4])
        self._MsgData.min_motor_current_limit_A0pk = convert_u32_to_float(data[5])
        temp = [0.0,convert_u32_to_float(data[6]),0.0,0.0]
        self._MsgData.mcu_inst_power_W = temp
        temp = [0.0,convert_u32_to_float(data[7]),0.0,0.0]
        self._MsgData.mcu_total_energy_Wh = temp
        
        self._MsgPub.publish(self._MsgData)
        self._seq += 1

class SIBOT_Dynamics:
    def __init__(self):
        self._MsgData = sibotDynamics()
        self._MsgPub = rospy.Publisher('/sibot/feedback/dynamics', sibotDynamics)
        self._MsgData.header.frame_id = ''
        
        self._TwistData = TwistWithCovarianceStamped()
        self._TwistPub = rospy.Publisher('/sibot/feedback/twist', TwistWithCovarianceStamped)
        self._TwistData.header.frame_id = '/sibot/base_link'
        self._TwistData.twist.covariance = [0.0] * 36
        self._TwistData.twist.covariance[0]  = 0.05
        self._TwistData.twist.covariance[7]  = 0.05
        self._TwistData.twist.covariance[14] = 0.05
        self._TwistData.twist.covariance[21] = 99999999.0
        self._TwistData.twist.covariance[28] = 99999999.0
        self._TwistData.twist.covariance[35] = 0.05
        
        self._seq = 0       
         
    def parse(self,data):
        self._MsgData.header.stamp = rospy.Time.now()
        self._MsgData.header.seq = self._seq

        self._TwistData.header.stamp = rospy.Time.now()
        self._TwistData.header.seq = self._seq  
        
        self._MsgData.vel_limit_mps = convert_u32_to_float(data[0])                            
        self._MsgData.yaw_rate_limit_rps = convert_u32_to_float(data[1])
        self._MsgData.linear_accel_mps2 = convert_u32_to_float(data[2]) 
        self._MsgData.linear_vel_mps = convert_u32_to_float(data[3]) 
        self._MsgData.differential_wheel_vel_rps = convert_u32_to_float(data[4])
        self._MsgData.right_wheel_vel_mps = convert_u32_to_float(data[5]) 
        self._MsgData.left_wheel_vel_mps = convert_u32_to_float(data[6])
        self._MsgData.right_wheel_pos_m = convert_u32_to_float(data[7])
        self._MsgData.left_wheel_pos_m = convert_u32_to_float(data[8]) 
        self._MsgData.linear_pos_m = convert_u32_to_float(data[9]) 
        self._MsgData.vel_target_mps = convert_u32_to_float(data[10]) 
        self._MsgData.yaw_rate_target_rps = convert_u32_to_float(data[11]) 
        self._MsgData.odom_est_x_m = convert_u32_to_float(data[12])
        self._MsgData.odom_est_y_m = convert_u32_to_float(data[13])
        self._MsgData.odom_est_yaw_angle_rad = convert_u32_to_float(data[14])
        
        
        self._TwistData.twist.twist.linear.x = self._MsgData.linear_vel_mps
        self._TwistData.twist.twist.linear.y = 0.0
        self._TwistData.twist.twist.linear.z = 0.0
        self._TwistData.twist.twist.angular.x = 0.0
        self._TwistData.twist.twist.angular.y = 0.0
        self._TwistData.twist.twist.angular.z = -self._MsgData.differential_wheel_vel_rps
        
        self._TwistPub.publish(self._TwistData)
        self._MsgPub.publish(self._MsgData)
        self._seq += 1  

class SIBOT_Configuration:
    def __init__(self):   
        self._MsgData = sibotConfiguration()
        self._MsgPub = rospy.Publisher('/sibot/feedback/configuration', sibotConfiguration)
        self._MsgData.header.frame_id = ''
        self._seq = 0    
        
    def parse(self,data):
        self._MsgData.header.stamp = rospy.Time.now()
        self._MsgData.header.seq = self._seq
        
        self._MsgData.vel_limit_mps = convert_u32_to_float(data[0])
        self._MsgData.accel_limit_mps2 = convert_u32_to_float(data[1])
        self._MsgData.decel_limit_mps2 = convert_u32_to_float(data[2])
        self._MsgData.dtz_decel_limit_mps2 = convert_u32_to_float(data[3])
        self._MsgData.coastdown_decel_mps2 = convert_u32_to_float(data[4])
        self._MsgData.yaw_rate_limit_rps = convert_u32_to_float(data[5])
        self._MsgData.yaw_accel_limit_rps2 = convert_u32_to_float(data[6])
        self._MsgData.tire_diameter_m = convert_u32_to_float(data[7])
        self._MsgData.wheelbase_length_m = convert_u32_to_float(data[8])
        self._MsgData.wheel_track_width_m = convert_u32_to_float(data[9])
        self._MsgData.transmission_ratio = convert_u32_to_float(data[10])
        self._MsgData.config_bitmap = data[11]
        self._MsgData.eth_ip_address = numToDottedQuad(data[12])
        self._MsgData.eth_port_number = data[13]
        self._MsgData.eth_subnet_mask = numToDottedQuad(data[14])
        self._MsgData.eth_gateway = numToDottedQuad(data[15])
        temp = [data[16],data[17],data[18],data[19]]
        self._MsgData.user_feedback_bitmaps = temp
        self._MsgData.lateral_accel_limit_mps2 = convert_u32_to_float(data[20])
        
        self._MsgPub.publish(self._MsgData)
        self._seq += 1  

class AHRS_Packet(object):
    def __init__(self):
        self.accel_mps2       = [0.0]*3
        self.gyro_rps         = [0.0]*3
        self.mag_T            = [0.0]*3
        self.rpy_rad          = [0.0]*3
        self.timestamp_sec    = 0.0
        self.linear_accel_covariance = 0.098 * 0.098
        self.angular_velocity_covariance = 0.012 * 0.012
        self.orientation_covariance = 0.035 * 0.035
        self.magnetic_field_covariance = 0.000002 * 0.000002
        
        self.publish_data = False
        self._MsgData = Imu()
        self._MsgPub = rospy.Publisher('/sibot/feedback/imu', Imu)
        self._MsgData.header.frame_id = '/sibot/microstrain_imu_frame'
        self._seq = 0
        
        self._MagMsgData = MagneticField()
        self._MagMsgPub = rospy.Publisher('/sibot/feedback/mag_feild', MagneticField)
        self._MagMsgData.header.frame_id = '/sibot/microstrain_imu_frame'
        self._Magseq = 0  
        

    def parse_data(self,data):
        index = 0
        for i in range(0,3):
            self.accel_mps2[i] = convert_u32_to_float(data[index]) * 9.81
            index+=1
        for i in range(0,3):
            self.gyro_rps[i] = convert_u32_to_float(data[index])
            index+=1
        for i in range(0,3):
            self.mag_T[i] = convert_u32_to_float(data[index]) * 0.0001
            index+=1
        for i in range(0,3):
            self.rpy_rad[i] = convert_u32_to_float(data[index])
            index+=1
        
        self.timestamp_sec = convert_float_to_u32(convert_u32_to_float(data[index]) * 62500)
        
        self._MsgData.header.stamp = rospy.Time.now()
        self._MagMsgData.header.stamp = rospy.Time.now()
        self._MsgData.header.seq = self._seq
        self._MagMsgData.header.seq = self._seq      
        
        roll = -self.rpy_rad[0] 
        pitch = self.rpy_rad[1]
        yaw = self.rpy_rad[2]
        while (yaw <= -math.pi):
            yaw += 2.0*math.pi;
        while (yaw > math.pi):
            yaw -= 2.0*math.pi;
        yaw*=-1.0;
        
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw, 'sxyz')
        
        self._MsgData.orientation.x = q[0]
        self._MsgData.orientation.y = q[1]
        self._MsgData.orientation.z = q[2]
        self._MsgData.orientation.w = q[3]
        
        self._MsgData.orientation_covariance[0] = self.orientation_covariance
        self._MsgData.orientation_covariance[4] = self.orientation_covariance
        self._MsgData.orientation_covariance[8] = self.orientation_covariance
        
        self._MsgData.linear_acceleration.x = -self.accel_mps2[0]
        self._MsgData.linear_acceleration.y = self.accel_mps2[1]
        self._MsgData.linear_acceleration.z = -self.accel_mps2[2]
        self._MsgData.linear_acceleration_covariance[0] = self.linear_accel_covariance
        self._MsgData.linear_acceleration_covariance[4] = self.linear_accel_covariance
        self._MsgData.linear_acceleration_covariance[8] = self.linear_accel_covariance
        
        self._MsgData.angular_velocity.x = -self.gyro_rps[0]
        self._MsgData.angular_velocity.y = self.gyro_rps[1]
        self._MsgData.angular_velocity.z = -self.gyro_rps[2]
        self._MsgData.angular_velocity_covariance[0] = self.angular_velocity_covariance
        self._MsgData.angular_velocity_covariance[4] = self.angular_velocity_covariance
        self._MsgData.angular_velocity_covariance[8] = self.angular_velocity_covariance
        
        self._MagMsgData.magnetic_field.x = -self.mag_T[0]
        self._MagMsgData.magnetic_field.y = self.mag_T[1]
        self._MagMsgData.magnetic_field.z = -self.mag_T[2]
        self._MagMsgData.magnetic_field_covariance[0] = self.magnetic_field_covariance
        self._MagMsgData.magnetic_field_covariance[4] = self.magnetic_field_covariance
        self._MagMsgData.magnetic_field_covariance[8] = self.magnetic_field_covariance
        
                
        if (True == self.publish_data):
            self._MsgPub.publish(self._MsgData)
            self._MagMsgPub.publish(self._MagMsgData)
            self._seq += 1


class GPS_Packet(object):
    def __init__(self):
        self.latitude_deg     = 0.0
        self.longitude_deg    = 0.0
        self.msl_height_m     = 0.0
        self.el_height_m      = 0.0
        self.horizontal_stdev = 0.0
        self.vertical_stdev   = 0.0
        self.valid_llh_flag   = 0
        
        self.fix_valid_flags        = 0
        self.fix_type               = 0
        self.fix_num_space_vehicles = 0
        
        self.hw_sensor_state   = 0
        self.hw_antenna_state  = 0
        self.hw_antenna_power  = 0
        self.hw_valid_hw_flags = 0
        
        self.publish_data = False
        
        self._HVMsgData = NavSatFix()
        self._HVMsgPub = rospy.Publisher('/sibot/feedback/gps/fix_3d', NavSatFix)
        self._HVMsgData.header.frame_id = '/sibot/gps_rcvr_frame'
        self._HVMsgData.status.service = NavSatStatus.SERVICE_GPS
        
        self._HMsgData = NavSatFix()
        self._HMsgPub = rospy.Publisher('/sibot/feedback/gps/fix_2d', NavSatFix)
        self._HMsgData.header.frame_id = '/sibot/gps_rcvr_frame'
        self._HMsgData.status.service = NavSatStatus.SERVICE_GPS
        self._seq = 0
        
        self.LAT_LON_FIX_VALID          = 0x0001
        self.ELLIPSOID_HEIGHT_FIX_VALID = 0x0002
        self.MSL_HEIGHT_FIX_VALID       = 0x0004
        self.HORIZONTAL_ACCURACY_VALID  = 0x0008
        self.VERTICAL_ACCURACY_VALID    = 0x0010

    def parse_data(self,data):
        
        self.latitude_deg     = convert_u64_to_double(data[0],data[1])
        self.longitude_deg    = convert_u64_to_double(data[2],data[3])
        self.el_height_m      = convert_u64_to_double(data[4],data[5])
        self.msl_height_m     = convert_u64_to_double(data[6],data[7])
        self.horizontal_stdev = convert_u32_to_float(data[8])
        self.vertical_stdev   = convert_u32_to_float(data[9])
        self.valid_llh_flag   = data[10]
        
        self.fix_valid_flags        = (data[11] & 0x0000FFFF) 
        self.fix_type               = (data[11] & 0xFF000000) >> 24 
        self.fix_num_space_vehicles = (data[11] & 0x00FF0000) >> 16
        
        self.hw_sensor_state   = (data[12] & 0xFF000000) >> 24
        self.hw_antenna_state  = (data[12] & 0x00FF0000) >> 16
        self.hw_antenna_power  = (data[12] & 0x0000FF00) >> 8
        self.hw_valid_hw_flags = (data[12] & 0x000000FF)
        
        self._HVMsgData.header.stamp = rospy.Time.now()
        self._HVMsgData.header.seq = self._seq
        
        self._HMsgData.header.stamp = rospy.Time.now()
        self._HMsgData.header.seq = self._seq
        
        self._HVMsgData.latitude = self.latitude_deg
        self._HVMsgData.longitude = self.longitude_deg
        self._HVMsgData.altitude = self.el_height_m
        
        self._HMsgData.latitude = self.latitude_deg
        self._HMsgData.longitude = self.longitude_deg
        self._HMsgData.altitude = 0.0      
        
        if (self.LAT_LON_FIX_VALID == (self.LAT_LON_FIX_VALID & self.valid_llh_flag)):
            self._HMsgData.status.status = NavSatStatus.STATUS_FIX
            if (self.ELLIPSOID_HEIGHT_FIX_VALID == (self.ELLIPSOID_HEIGHT_FIX_VALID & self.valid_llh_flag)):
                self._HVMsgData.status.status = NavSatStatus.STATUS_FIX
            else:
                self._HVMsgData.status.status = NavSatStatus.STATUS_NO_FIX
        else:
            self._HVMsgData.status.status = NavSatStatus.STATUS_NO_FIX
            self._HMsgData.status.status = NavSatStatus.STATUS_NO_FIX             
        
        if (self.HORIZONTAL_ACCURACY_VALID == (self.HORIZONTAL_ACCURACY_VALID & self.valid_llh_flag)):
            self._HMsgData.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self._HMsgData.position_covariance[0] = self.horizontal_stdev * self.horizontal_stdev
            self._HMsgData.position_covariance[4] = self.horizontal_stdev * self.horizontal_stdev
            self._HMsgData.position_covariance[8] = 100.0
            
            if (self.VERTICAL_ACCURACY_VALID == (self.VERTICAL_ACCURACY_VALID & self.valid_llh_flag)):
                self._HVMsgData.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                self._HMsgData.position_covariance[0] = self.horizontal_stdev * self.horizontal_stdev
                self._HMsgData.position_covariance[4] = self.horizontal_stdev * self.horizontal_stdev
                self._HMsgData.position_covariance[8] = self.vertical_stdev * self.vertical_stdev
            else:
                self._HVMsgData.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        else:
            self._HVMsgData.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            self._HMsgData.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            
        if (True == self.publish_data):
            self._HMsgPub.publish(self._HMsgData)
            self._HVMsgPub.publish(self._HVMsgData)    
            self._seq+=1            
        
        
class SIBOT_IMU(object):
    def __init__(self):
        self.status = 0
        self.errors = 0
        self.ahrs = AHRS_Packet()
        self.gps  = GPS_Packet()
        self.first_time = True
        self.waitfor_ekf = rospy.get_param('~set_ekf_initial_pose',False)
    def parse_data(self,data):
        self.ahrs.parse_data(data[0:13])
        self.gps.parse_data(data[13:26]) 
        self.status = data[26] & 0xFF
        self.missed_ahrs_messages = (self.status & 0xFF000000) >> 24
        self.missed_gps_messages = (self.status & 0x00FF0000) >> 16
        
        if (2 == self.status):
            self.ahrs.publish_data = True
            self.gps.publish_data = True
            
            if (self.waitfor_ekf == True):
                if (True == self.first_time):
                    temp = PoseWithCovarianceStamped()
                    set_pose = rospy.ServiceProxy('set_pose',SetPose) 
                    try:
                        rospy.wait_for_service('set_pose')
                        temp.pose.pose.position.x = 0.0
                        temp.pose.pose.position.y = 0.0
                        temp.pose.pose.position.z = 0.0
                        temp.pose.pose.orientation = self.ahrs._MsgData.orientation
                        set_pose(temp)
                        rospy.loginfo("Set EKF initial pose with heading")
                        self.first_time = False
                        
                    except:
                        rospy.logwarn("Could not find /set_pose service from robot_localization!")
                        rospy.logwarn("Call set_pose service elsewhere to initialize pose.....")
                        self.first_time = False
                        
                                   


class SIBOT_DATA:
    def __init__(self):
        self.imu = SIBOT_IMU()
        self.status = SIBOT_Status()
        self.battery = SIBOT_Batteries()
        self.config_param = SIBOT_Configuration()
        self.propulsion = SIBOT_Propulsion()
        self.dynamics = SIBOT_Dynamics()
    
