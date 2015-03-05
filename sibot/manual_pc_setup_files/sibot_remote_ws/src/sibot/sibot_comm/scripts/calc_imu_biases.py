#!/usr/bin/env python
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
 
 \file   calc_imu_biases.py

 \brief  runs the bias calculator

 \Platform: Linux/ROS Hydro
--------------------------------------------------------------------"""
import rospy,tf,math
from sensor_msgs.msg import Imu


class IMU_BIAS:
    def __init__(self):

        self.imu_acc_sum=[0.0]*3
        self.imu_gyro_sum=[0.0]*3
        self.imu_est_sum=[0.0]*2
        self.imu_acc_bias=[0.0]*3
        self.imu_gyro_bias=[0.0]*3
        self.imu_est_bias=[0.0]*2
        self.samples = 0
        self.run = True

    def imu_cb(self,data):
    
        if not self.run:
            return

        self.samples+=1.0
        self.imu_acc_sum[0] += data.linear_acceleration.x
        self.imu_acc_sum[1] += data.linear_acceleration.y
        self.imu_acc_sum[2] += data.linear_acceleration.z
        
        
        self.imu_acc_bias[0] =  (self.imu_acc_sum[0]/self.samples)
        self.imu_acc_bias[1] =  (self.imu_acc_sum[1]/self.samples)
        self.imu_acc_bias[2] =  ((self.imu_acc_sum[2]/self.samples) - 9.81)
        
        
        self.imu_gyro_sum[0] += data.angular_velocity.x
        self.imu_gyro_sum[1] += data.angular_velocity.y
        self.imu_gyro_sum[2] += data.angular_velocity.z
        
        self.imu_gyro_bias[0] =  self.imu_gyro_sum[0]/self.samples
        self.imu_gyro_bias[1] =  self.imu_gyro_sum[1]/self.samples
        self.imu_gyro_bias[2] =  self.imu_gyro_sum[2]/self.samples
        
        
        q = [0.0]*4
        q[0] = data.orientation.x
        q[1] = data.orientation.y
        q[2] = data.orientation.z
        q[3] = data.orientation.w
        rpy = tf.transformations.euler_from_quaternion(q)
        
        
        rospy.loginfo("roll: %f"%(rpy[0] * (180.0/math.pi)))
        rospy.loginfo("pitch: %f"%(rpy[1] * (180.0/math.pi))) 
        rospy.loginfo("yaw: %f"%(rpy[2] * (180.0/math.pi)))
        
        
        self.imu_est_sum[0] += rpy[0]
        self.imu_est_sum[1] += rpy[1]
        
        self.imu_est_bias[0] = self.imu_est_sum[0]/self.samples
        self.imu_est_bias[1] = self.imu_est_sum[1]/self.samples
        
        
        if self.samples > 6000:
        
            outfile_path='/home/sibot/nav_ws/src/sibot/sibot_comm/config/sibot_imu_biases.yaml'
            outfile=open(outfile_path,'w')
            
            outfile.write("acc_x_bias: %.15f\n"%self.imu_acc_bias[0])
            outfile.write("acc_y_bias: %.15f\n"%self.imu_acc_bias[1]) 
            outfile.write("acc_z_bias: %.15f\n"%self.imu_acc_bias[2])
            
            outfile.write("gyro_x_bias: %.15f\n"%self.imu_gyro_bias[0])
            outfile.write("gyro_y_bias: %.15f\n"%self.imu_gyro_bias[1]) 
            outfile.write("gyro_z_bias: %.15f\n"%self.imu_gyro_bias[2])

            outfile.write("roll_offset: %.15f\n"%self.imu_est_bias[0])
            outfile.write("pitch_offset: %.15f\n"%self.imu_est_bias[1])
            
            rospy.loginfo("roll_offset: %.15f"%self.imu_est_bias[0])
            rospy.loginfo("pitch_offset: %.15f"%self.imu_est_bias[1])             
            rospy.loginfo("acc_x_bias: %.15f"%self.imu_acc_bias[0])
            rospy.loginfo("acc_y_bias: %.15f"%self.imu_acc_bias[1])
            rospy.loginfo("acc_z_bias: %.15f"%self.imu_acc_bias[2])
            rospy.loginfo("gyro_x_bias: %.15f"%self.imu_gyro_bias[0])
            rospy.loginfo("gyro_y_bias: %.15f"%self.imu_gyro_bias[1])
            rospy.loginfo("gyro_z_bias: %.15f\n"%self.imu_gyro_bias[2])
            outfile.close()
            self.run = False
            
      
    

    

if __name__ == "__main__":
    rospy.init_node('calc_imu_biases')
    imu_bias_calc = IMU_BIAS()
    rospy.Subscriber("/sibot/feedback/imu",Imu,imu_bias_calc.imu_cb)
    
    rospy.spin()
    
    
    
    
    
    
    
    
    
    



