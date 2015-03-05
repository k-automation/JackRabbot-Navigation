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
 
 \file   rmp_comm.py

 \brief  runs the driver

 \Platform: Linux/ROS Hydro
--------------------------------------------------------------------"""
import roslib; roslib.load_manifest('sibot_comm')
from system_defines import *
from utils import *
from rmp_interface import RMP
from sibot_comm.msg import *
from geometry_msgs.msg import Twist
import sys,time,threading,Queue,rospy,math

"""
Define the update delay or update period in seconds. Must be greater
than the minimum of 0.01s
"""
UPDATE_DELAY_SEC = 0.01

"""
Define some general parameters for the example like various commands 
"""
RMP_NULL_CMD = [RMP_CFG_CMD_ID,RMP_CMD_NONE,0]

class RMPThread:
    def __init__(self):
        """
        Initialize the node and subscribe to the command topic
        """
        rospy.init_node('sibot_comm')
        self.platform = rospy.get_param('~platform',None)
        self.log_data = rospy.get_param('~log_data',False)
        self.cmd_update_rate = rospy.get_param('~cmd_update_rate',0.01)
        
        if (self.cmd_update_rate < 0.01):
            rospy.logerr("Update rate is too fast, 100Hz maximum")
            sys.exit(0)
            return
        
        if (True == self.log_data):
            rospy.loginfo("Logging data in ~/.ros/RMP_DATA_LOGS")

        if (None == self.platform):
            rospy.logerr("You must define the platform type parameter")
            sys.exit(0)
            return
        elif ('sibot' != self.platform):
            rospy.logerr("Bad platform parameter, not supported by this driver, contact info@stanleyinnovation.com to get the correct drivers")
            sys.exit(0)
            return
        else:       
            import_filename = self.platform + '_config_params' 
        
        try:        
            self._cfg = __import__(import_filename, globals(), locals(), ['*'], -1)
        except:
            rospy.logerr("platform %s not supported, exiting...", self.platform)
            sys.exit(0)
            return
        
        """
        Get the configuration for the platform
        """    
        if ('rmp210' == self.platform ):
            configs = self._cfg.rmp210_cfg_params
        elif ('rmp220' == self.platform ):
            configs = self._cfg.rmp220_cfg_params
        elif ('rmp440' == self.platform ):
            configs = self._cfg.rmp440_cfg_params
        elif ('omni' == self.platform ):
            configs = self._cfg.omni_cfg_params
        elif ('arti' == self.platform ):
            configs = self._cfg.arti_cfg_params
        elif ('sibot' == self.platform ):
            configs = self._cfg.sibot_cfg_params
                                            
        """
        Get the configuration parameters
        """
        self.cfg_params = [0] * NUMBER_OF_NVM_CONFIG_PARAMS
        for i in range(0,NUMBER_OF_NVM_CONFIG_PARAMS):
            self.cfg_params[i] = [RMP_CFG_CMD_ID,(i+1),configs[i]]
            
        """
        Get the limits for steering and throttle
        """
        self.vel_limit = convert_u32_to_float(self.cfg_params[RMP_CMD_SET_MAXIMUM_VELOCITY-1][2])
        self.steer_limit = convert_u32_to_float(self.cfg_params[RMP_CMD_SET_MAXIMUM_TURN_RATE-1][2])

        """
        The platform address may be different than the one in your config
        (rmp_config_params.py). This would be the case if you wanted to update 
        ethernet configuration. If the ethernet configuration is updated the
        system needs to be power cycled for it to take effect and this should
        be changed to match the new values you defined in your config
        """
        rmp_ip = numToDottedQuad(self.cfg_params[RMP_CMD_SET_ETH_IP_ADDRESS-1][2])
        rmp_port = self.cfg_params[RMP_CMD_SET_ETH_PORT_NUMBER-1][2]
        rmp_addr = (rmp_ip,rmp_port)
        
        """
        Create and response and command queue. The responses will be in the form of 
        a dictionary containing the vaiable name as the key and a converted value
        the names are defined in the feedback_X_bitmap_menu_items dictionaries if a particular
        variable is of interest
        """
        self.rsp_queue = Queue.Queue()
        self.cmd_queue = Queue.Queue()
        self.in_flags  = Queue.Queue()
        self.out_flags = Queue.Queue()
        
        """
        Create the thread to run RMP
        """
        self.my_thread = threading.Thread(target=RMP, args=(self.platform,self.cfg_params,rmp_addr,self.rsp_queue,self.cmd_queue,self.in_flags,self.out_flags,self.cmd_update_rate))
        self.my_thread.daemon = True
        self.my_thread.start()
        
        """
        Initialize my event handler class
        """
        """
        Flag to run the loop
        """    
        self._continue = True
        self.cmd_queue_buffer = Queue.Queue()
        self.last_time = 0.0

        """
        This is the dictionary that the outflags get passed to. Each one can be
        redefined to be passed to whatever user function you would like
        """
        self.handle_event = dict({RMP_KILL:sys.exit,
                                  RMP_INIT_FAILED:self.InitFailedExit,
                                  RMP_IS_DEAD:self.Kill_loop,
                                  RMP_TX_RDY:self.Send_Cmd,
                                  RMP_RSP_DATA_RDY:self.Get_Rsp})
        

        rospy.Subscriber("/sibot/cmd_vel", Twist, self.add_motion_command_to_queue)
        rospy.Subscriber("/rmp_command",rmpConfigCmd,self.add_config_command_to_queue)
        
        
    def __del__(self):
        self.Kill_loop()
    
    """
    Define the main function for the example. It creates a thread to run RMP and handles
    passing the events to the user defined handlers in user_event_handlers.py
    """
    def rmp_thread(self):
        """
        -------------------------------------------------------------------------------
        User loop starts here modify to make it do what you want. 
        
        You can pipe std_in from another application to the command queue and the response to std out or 
        let the event handlers define everything. That is up to the user. In this example we transition modes, 
        send motion commands (zeroed), play audio songs, and print the response dictionary. The application 
        terminates the thread and exits when all the songs have been played. It is just an example of how to 
        spawn a RMP thread, handle events, and send/receive data
        ------------------------------------------------------------------------------- 
        """
    
        """
        Run until signaled to stop
        Perform the actions defined based on the flags passed out
        """
        while ((True == self._continue) and (False == rospy.is_shutdown())):
            self.handle_event[self.out_flags.get()]()
        
        """
        Kill the thread
        """
        self.in_flags.put(RMP_KILL)
        
        """
        Wait for the thread to die
        """
        while self.my_thread.isAlive():
            pass
        
        """
        Exit main
        """
        rospy.spin()
        
    def Send_Cmd(self):
        if not (self.cmd_queue_buffer.empty()):
            self.cmd_queue.put(self.cmd_queue_buffer.get())
        rospy.logdebug("sibot cmd received")    
                        
    def Get_Rsp(self):
        rospy.logdebug("sibot feedback received")

    def InitFailedExit(self):
        rospy.loginfo("RMP initialization failed....") 
        rospy.loginfo("exiting.....")
        self.in_flags.put(RMP_KILL)
        self._continue = False
        
    def Kill_loop(self):
        rospy.loginfo("Loop terminated, killing RMP thread and exiting.....")
        self.in_flags.put(RMP_KILL)
        self._continue = False
        
    def add_motion_command_to_queue(self,command):
        
        """
        Check for correct parameters and populate command array to be sent to rmp 
        Velocity parameters outside of allowable range will be set to max or min limit
        """
        
        if ('omni' != self.platform):
            cmds = [0] * 3
            cmds[0] = RMP_MOTION_CMD_ID
            cmds[1] = limit_f((command.linear.x/self.vel_limit),1.0)
            cmds[2] = -1.0 * limit_f((command.angular.z/self.steer_limit),1.0)        
        else:
            x              = limit_f((command.linear.x/self.vel_limit),1.0)
            y              = limit_f((command.linear.y/self.vel_limit),1.0)
            velocity_cmd   = math.sqrt(math.pow(x,2) + math.pow(y,2))
            velocity_cmd   = clamp_value_f(velocity_cmd,0.0,1.0)
            yaw_cmd        = limit_f((command.angular.z/self.steer_limit),1.0) 
            angle_cmd      = math.atan2(y,x) * (180.0/math.pi)
            
            if (angle_cmd < 0.0):
                angle_cmd = 360 + angle_cmd
            cmds = [RMP_OMNI_MOTION_CMD_ID,velocity_cmd,yaw_cmd,angle_cmd]
        
        self.cmd_queue_buffer.put(cmds)
            
    def add_config_command_to_queue(self,command):
        
        self.last_time = time.clock()
        if (RMP_CFG_CMD_ID == command.header):
            cmds = [0] * 3
            cmds[0] = RMP_CFG_CMD_ID
            cmds[1] = command_ids[command.id]
            
            """
            Determine if the param is an int or a float. Test for int first and send the float on exception
            it is up to the publisher to verify the command parameter is valid
            """
            try:
                temp = int_command_params[command.id]
                cmds[2] = command.int_config_val
            except:
                cmds[2] = convert_float_to_u32(command.float_config_val)

            
            self.cmd_queue_buffer.put(cmds)
            
        else:
            print "Incorrect header. Publish motion commands to geometry_msgs/Twist. Command not sent."
            return

if __name__ == "__main__":       
    rmp = RMPThread()
    rmp.rmp_thread()    
    
