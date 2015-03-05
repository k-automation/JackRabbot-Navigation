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
 
 \file   rmp_interface.py

 \brief  This module contains the interface to the RMP

 \Platform: Linux/ROS Hydro
--------------------------------------------------------------------"""
from crc16  import *
from utils import *
from system_defines import *
from rmp_data_classes import *
from io_eth_cmd import IO_ETHERNET
import time,os,sys,rospy


"""
Define the variable for fixed point Q15 format for
converting certain parameters for OMNI
"""
Q15             = 32767

"""
Main class for the RMP interface
"""
class RMP:
    def __init__(self,platform,cfgs,rmp_addr,rsp_queue,cmd_queue,in_flags,out_flags,update_rate=MIN_UPDATE_PERIOD_SEC):
        
        self.platform = platform
        if ('rmp210' == self.platform ) or ('rmp220' == self.platform ) or ('rmp440' == self.platform ) or ('omni' == self.platform ):
            import_filename = 'rmp_config_params'
        else:
            import_filename = self.platform + '_config_params'
        
        self._cfg = __import__(import_filename, globals(), locals(), ['*'], -1)
        
        """
        SIBOT Data classes
        """
        self.sibot_data = SIBOT_DATA()

        """
        generate the CRC table
        """
        generate_crc_table()
        
        """
        Initialize the bitmaps and create a dictionary for holding the
        user defined feedback data
        """
        self.bitmap = [0]*4
        self.bitmap[0] = cfgs[RMP_CMD_SET_USER_FB_1_BITMAP-1][2]
        self.bitmap[1] = cfgs[RMP_CMD_SET_USER_FB_2_BITMAP-1][2]
        self.bitmap[2] = cfgs[RMP_CMD_SET_USER_FB_3_BITMAP-1][2]
        self.bitmap[3] = cfgs[RMP_CMD_SET_USER_FB_4_BITMAP-1][2]
        
        """
        Update expected number of feedback words
        """
        item = 0
        for x in range(0,4):
            for i in range(0,32):
                if (self.bitmap[x] & (1<<i)):
                    item += 1
        """
        Add one for the CRC
        """
        self.expected_items = item+1
        
        """
        Get the RMP address
        """
        self.comm = IO_ETHERNET(rmp_addr)
        
        self.in_flags = in_flags
        self.out_flags = out_flags
        
        if (update_rate >= MIN_UPDATE_PERIOD_SEC):
            self.delay = update_rate
        else:
            rospy.loginfo("Bad Update Period needs to be longer than 0.01s.....")
            rospy.loginfo("exiting......")
            self.out_flags.put(RMP_INIT_FAILED)
            self.Close()
                    
        if (False == self.comm.success):
            rospy.loginfo("Could not connect to RMP UDP socket.....")
            rospy.loginfo("exiting......")
            self.out_flags.put(RMP_INIT_FAILED)
            self.Close()
            
        if (False == self.set_and_verify_config_params(cfgs)):
            rospy.loginfo("Could not configure RMP......")
            rospy.loginfo("exiting......")
            self.out_flags.put(RMP_INIT_FAILED)
            self.Close()
            
        if (False == self.goto_tractor_and_indicate()):
            rospy.loginfo("Could not set initial state RMP......")
            rospy.loginfo("exiting......")
            self.out_flags.put(RMP_INIT_FAILED)
            self.Close()
        
        """
        Get the queues and the time stamps now that initialization was successful
        """
        self.cmd_queue = cmd_queue
        self.rsp_queue = rsp_queue
        self.last_update_time = rospy.Time.now().to_sec()
        
        
            
        """
        Run the thread
        """
        self.run()
        
    def run(self):
        while not rospy.is_shutdown():
            
            """
            Check the flags coming in. Presently there is only a kill.
            """
            while not self.in_flags.empty():
                if (RMP_KILL == self.in_flags.get()):
                    rospy.loginfo("RMP thread has been killed by application......")
                    rospy.loginfo("exiting.........")
                    self.out_flags.put(RMP_IS_DEAD)
                    self.Close()
                    sys.exit()
                               
            """
            Check for data each time and put it in the queue if it exists.
            """
            data = self.comm.Receive(self.expected_items)
            d = rospy.Duration.from_sec(0.005)
            if (self.update_feedback(data,False)):
                if not self.cmd_queue.empty():
                    self.update_rmp_commands(self.cmd_queue.get())
                self.out_flags.put(RMP_TX_RDY)
                self.out_flags.put(RMP_RSP_DATA_RDY)
            rospy.sleep(d)
            
        self.Close()
        sys.exit()

    def set_and_verify_config_params(self,config):
        """
        The commands that force the feedback array to just contain the configurable elements
        but not update the UDFB to do so. This allows the user to verify the configuration
        while it is changing.
        """
        force_nvm_feedback = [RMP_CFG_CMD_ID,RMP_CMD_FORCE_CONFIG_FEEDBACK_BITMAPS,1]
        set_user_feedback = [RMP_CFG_CMD_ID,RMP_CMD_FORCE_CONFIG_FEEDBACK_BITMAPS,0]
        set_cont_data_feedback = [RMP_CFG_CMD_ID,36,1]
        
        
        """
        Start by sending the force config feedback command and check for the
        appropriate response
        """
        attempts = 0
        success = False
        while ((False == success) and (attempts<10)):
            self.update_rmp_commands(force_nvm_feedback)
            time.sleep(0.1)
            loaded_params = self.comm.Receive(FORCED_CONFIG_FEEDBACK_ITEMS)
            if (None != loaded_params):
                success = True
            else:
                attempts += 1
        
        """
        If the command to force the config feedback was successful check to see if there
        are non-matching configuration parameters
        """
        if (False == success):
            print "Could not set RMP_CMD_FORCE_CONFIG_FEEDBACK_BITMAPS....."
            print "The platform did not respond, ensure it is operational and the IP address is correct...."
            return False
        else:
            non_matching_params = []
            for i in range(0,NUMBER_OF_NVM_CONFIG_PARAMS):
                if (loaded_params[i] != config[i][2]):
                    non_matching_params.append(i)
        
        """
        Load each configuration parameter which does not match the configuration file
        """
        attempts = 0    
        for i in range(0,len(non_matching_params)):
            idx = non_matching_params[i]
            success = False
            while ((False == success) and (attempts<10)):
                attempts+=1
                self.update_rmp_commands(config[idx])
                time.sleep(0.1)
                loaded_params = self.comm.Receive(FORCED_CONFIG_FEEDBACK_ITEMS)
                if (None != loaded_params): 
                    if (loaded_params[idx] == config[idx][2]):
                        success = True
                        attempts = 0
            if (False == success):
                break
        
        """
        Notify the user if we could not set the parameter
        """
        if (False == success):
            rospy.loginfo("Could not set param %(1)s....." %{"1":command_ids[idx+1]})
            rospy.loginfo("The parameter is likely not valid, check it in rmp_config_params.py")
            return False
        
        """
        Switch back to the user feedback array and make sure we get an appropriate response
        """
        attempts = 0
        success = False
        while ((False == success) and (attempts<10)):
            self.update_rmp_commands(set_user_feedback)
            time.sleep(0.1)
            data = self.comm.Receive(self.expected_items)
            if (None != data):
                success = True
            else:
                attempts += 1
                
        """
        Switch back to the user feedback array and make sure we get an appropriate response
        """
        attempts = 0
        success = False
        while ((False == success) and (attempts<10)):
            self.update_rmp_commands(set_cont_data_feedback)
            time.sleep(0.01)
            data = self.comm.Receive(self.expected_items)
            if (None != data):
                success = True
            else:
                attempts += 1
                
                    
        """
        Could not reset the feedback 
        """
        if (False == success):
            rospy.loginfo("Could not set user defined feedback" %{"1":idx+1})
            rospy.loginfo("The platform did not respond, ")
            return False
        
        return True
        
        
    def goto_tractor_and_indicate(self):
        """
        define the commands for the function
        """
        
        set_tractor_mode = [RMP_CFG_CMD_ID,RMP_CMD_SET_OPERATIONAL_MODE,TRACTOR_REQUEST]
        send_no_command = [RMP_CFG_CMD_ID,RMP_CMD_NONE,0]
        set_tractor_mode_indicator = [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_TEST_SWEEP]
        
        """
        Send Tractor mode and verify
        """
        if (False == self.send_single_command(set_tractor_mode,index=6,verify_val=4)):
            rospy.loginfo("Could not set Tractor Mode")
            rospy.loginfo("The platform did not respond, ")
            return False
            
        """
        Wait for transition to complete
        """
        d = rospy.Duration.from_sec(2.0)
        rospy.sleep(d)
        
        """
        Send the audio command
        """
        self.send_single_command(set_tractor_mode_indicator,verify=False,tries=200, timeout_val=4.0)
        
        """
        clear out any data left in the receive
        """
        flush = True
        while (flush):
            data = self.comm.Receive(self.expected_items)
            if (None == data):
                flush = False
            
        return True
        
    def send_single_command(self, cmd, index=0, verify=True, verify_val=None, tries=10, timeout_val=0.2):
        """
        Send yaw accel low and verify
        """
        attempts = 0
        success = False
        timeout = False
        start_time = rospy.Time.now().to_sec()
        while ((False == success) and (attempts<tries) and (False == timeout)):
        
            """
            Check for correct value
            """
            data = self.comm.Receive(self.expected_items)
            d = rospy.Duration.from_sec(0.005)
            
            if ((rospy.Time.now().to_sec() - start_time) > timeout_val):
                timeout = True
            
            if (self.update_feedback(data,False)):
                self.update_rmp_commands(cmd)
                
                if (False == verify):
                     
                    attempts += 1
                    if (attempts >= tries):
                        success = True
                        attempts = 0
                        timeout = False
                else:
                    
                    
                    if (None == verify_val):
                        verify_val = cmd[2]
                    
                    if (verify_val == data[index]):
                        success = True
                        attempts = 0
                        timeout = False
                    else:
                        attempts += 1

            rospy.sleep(d)
                    
        """
        Could not set value 
        """
        if (False == success) or (True == timeout):
            return False
            
        return True    

    def update_rmp_commands(self,input_cmd):
       
        """
        Populate the message to the RMP platform if it is not a
        valid format return False
        """
        try:
            cmds = [0]*3
            cmds[0] = input_cmd[0]
            send_cmd = True
            
            if (cmds[0] == RMP_OMNI_MOTION_CMD_ID):
                vel_cmd = int((input_cmd[1] * Q15))
                yaw_cmd = int((input_cmd[2] * Q15))
                cmds[1]= (((vel_cmd << 16) & 0xFFFF0000) | (yaw_cmd & 0x0000FFFF))
                cmds[2] = int(convert_float_to_u32(input_cmd[3]))
            elif (cmds[0] == RMP_MOTION_CMD_ID):
                cmds[1] = int(convert_float_to_u32(input_cmd[1]))
                cmds[2] = int(convert_float_to_u32(input_cmd[2]))
            elif (cmds[0] == RMP_CFG_CMD_ID):
                cmds[1] = int(input_cmd[1])
                cmds[2] = int(input_cmd[2])
            else:
                send_cmd = False
        except:
            send_cmd = False
            
        """
        If we have a valid command send it 
        """
        if (True == send_cmd):
                
            output = self.Convert_RMP_Cmds_for_Serial_Interface(cmds)
            self.comm.Send(output)
            
        return send_cmd
    
    def update_feedback(self,data=None,init=False):
        ret = True
        
        """
        If we have data convert and publish it
        """  
        if (data != None):
            try:
                self.sibot_data.status.parse(data[0:8])
                self.sibot_data.battery.parse(data[8:16])
                self.sibot_data.propulsion.parse(data[16:24])
                self.sibot_data.dynamics.parse(data[24:39])
                self.sibot_data.config_param.parse(data[39:60])
                self.sibot_data.imu.parse_data(data[60:88])
            except:
                rospy.logerr("could not publish to a closed topic")
        else:
            ret = False
        
        return ret
            
    def Close(self):
        self.comm.Close()
        sys.exit()
            
    def Convert_RMP_Cmds_for_Serial_Interface(self,cmds):
        """
        Convert a set of commands for the UDP Ethernet interface
        """
        rmp_cmd = [0]*NUM_USB_ETH_BYTES;
        
        rmp_cmd[RMP_USB_ETH_CAN_ID_HIGH_INDEX] = int((cmds[0] & 0xFF00) >> 8)
        rmp_cmd[RMP_USB_ETH_CAN_ID_LOW_INDEX]  = int((cmds[0] & 0x00FF))
        rmp_cmd[RMP_USB_ETH_CAN_DATA_0_INDEX]  = int((cmds[1] & 0xFF000000) >> 24)
        rmp_cmd[RMP_USB_ETH_CAN_DATA_1_INDEX]  = int((cmds[1] & 0x00FF0000) >> 16)
        rmp_cmd[RMP_USB_ETH_CAN_DATA_2_INDEX]  = int((cmds[1] & 0x0000FF00) >> 8)
        rmp_cmd[RMP_USB_ETH_CAN_DATA_3_INDEX]  = int((cmds[1] & 0x000000FF))
        rmp_cmd[RMP_USB_ETH_CAN_DATA_4_INDEX]  = int((cmds[2] & 0xFF000000) >> 24)
        rmp_cmd[RMP_USB_ETH_CAN_DATA_5_INDEX]  = int((cmds[2] & 0x00FF0000) >> 16)
        rmp_cmd[RMP_USB_ETH_CAN_DATA_6_INDEX]  = int((cmds[2] & 0x0000FF00) >> 8)
        rmp_cmd[RMP_USB_ETH_CAN_DATA_7_INDEX]  = int((cmds[2] & 0x000000FF))
        
        """
        Compute the CRC for the command 
        """
        compute_buffer_crc(rmp_cmd,NUM_USB_ETH_BYTES)
        
        """
        Convert the string to char data and return it
        """
        rmp_cmd_chars = []
        for x in range(0,len(rmp_cmd)):
            rmp_cmd_chars.append(chr(rmp_cmd[x]))   
        
        output = ''.join(rmp_cmd_chars)
        
        return output        
        
        
    
