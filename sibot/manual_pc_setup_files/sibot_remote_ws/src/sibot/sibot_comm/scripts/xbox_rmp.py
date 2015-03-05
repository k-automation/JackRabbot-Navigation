#!/usr/bin/env python
import roslib 
roslib.load_manifest('sibot_comm')
import rospy,sys
from utils import *
from system_defines import *

from sibot_comm.msg import *
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

BUTTON_A      = 0
BUTTON_B      = 1
BUTTON_X      = 2
BUTTON_Y      = 3
BUTTON_LB     = 4
BUTTON_RB     = 5
BUTTON_BACK   = 6
BUTTON_START  = 7
BUTTON_POWER  = 8
BUTTON_LSTICK = 9
BUTTON_RSTICK = 10
BUTTON_DPAD_L = 11
BUTTON_DPAD_R = 12
BUTTON_DPAD_U = 13
BUTTON_DPAD_D = 14

AXIS_LR_LSTICK = 0
AXIS_UD_LSTICK = 1
AXIS_L_TRIGGER = 2
AXIS_LR_RSTICK = 3
AXIS_UD_RSTICK = 4
AXIS_R_TRIGGER = 5

class XBoxRMP:
    def __init__(self):
        rospy.init_node('xbox_to_rmp')
        self.platform = rospy.get_param('~platform',None)
        
        if (None == self.platform):
            sys.exit(0)
            return
        elif ('sibot' != self.platform):
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
        if ('sibot' == self.platform ):
            configs = self._cfg.sibot_cfg_params
        else:
            rospy.logerr("platform %s not supported, exiting...", self.platform)
            sys.exit(0)
            return        
            
        """
        Get the limits for steering and throttle
        """
        self.vel_limit = convert_u32_to_float(configs[RMP_CMD_SET_MAXIMUM_VELOCITY-1])
        self.steer_limit = convert_u32_to_float(configs[RMP_CMD_SET_MAXIMUM_TURN_RATE-1])
        self.first_time = True
        self.send_cmd_none = False
        self.no_motion_commands = True
        self.last_motion_command_time = rospy.Time.now().to_sec()
            
        self.cfg_cmd = rmpConfigCmd()
        self.cfg_pub = rospy.Publisher('rmp_command', rmpConfigCmd)
        self.goalrecorder_pub = rospy.Publisher('/sibot/teleop/record_pose',Bool)
        
        self.motion_cmd = Twist()
        self.motion_pub = rospy.Publisher('cmd_vel', Twist)
        
        self.deadman_cmd = Bool()
        self.deadman_pub = rospy.Publisher('joy_deadman',Bool)
        self.goalrecorded = False 

    def xbox_to_rmp(self, joyMessage):
    
        if (True == self.first_time):
            self.num_buttons = len(joyMessage.buttons)
            self.db_cnt = [0] * self.num_buttons
            self.button_state = [0] * self.num_buttons
            self.first_time = False
              
        for i in range(0,self.num_buttons):
        
            if (1 == joyMessage.buttons[i]):
                self.db_cnt[i]+=1
                if (self.db_cnt[i] > 25):
                    self.db_cnt[i] = 25
                    self.button_state[i] = 1
            else:
                self.button_state[i] = 0
                self.db_cnt[i] = 0
                
        
        if self.button_state[BUTTON_DPAD_D] == 1:
            
            if (False == self.goalrecorded):
                temp = Bool()
                temp.data = True
                self.goalrecorder_pub.publish(temp)
                self.goalrecorded= True
        else:
            self.goalrecorded= False
            
                
        self.cfg_cmd.header = RMP_CFG_CMD_ID
        # A button pressed triggers tractor mode command
        if self.button_state[BUTTON_LB] == 1:
            self.cfg_cmd.id = 'RMP_CMD_SET_OPERATIONAL_MODE'
            self.cfg_cmd.int_config_val = DTZ_REQUEST
        #elif self.button_state[BUTTON_RB] == 1:
            #self.cfg_cmd.id = 'RMP_CMD_SET_OPERATIONAL_MODE'
            #self.cfg_cmd.int_config_val = DTZ_REQUEST
        # X button pressed triggers shutdown command
        elif self.button_state[BUTTON_POWER] == 1:
            self.cfg_cmd.id = 'RMP_CMD_SET_OPERATIONAL_MODE'
            self.cfg_cmd.int_config_val = POWERDOWN_REQUEST
        elif self.button_state[BUTTON_X] == 1:
            self.cfg_cmd.id = 'RMP_CMD_SET_OPERATIONAL_MODE'
            self.cfg_cmd.int_config_val = TRACTOR_REQUEST
        # Y button pressed triggers standby mode command
        elif self.button_state[BUTTON_B] == 1:
            self.cfg_cmd.id = 'RMP_CMD_SET_OPERATIONAL_MODE'
            self.cfg_cmd.int_config_val = STANDBY_REQUEST
        elif self.button_state[BUTTON_BACK] == 1:
            self.cfg_cmd.id = 'RMP_CMD_RESET_INTEGRATORS'
            self.cfg_cmd.int_config_val = RESET_ALL_POSITION_DATA
        else:
            self.cfg_cmd.id = 'RMP_CMD_NONE'
            self.cfg_cmd.int_config_val = 0
            
        if ('RMP_CMD_NONE' != self.cfg_cmd.id):
            self.cfg_pub.publish(self.cfg_cmd)
            self.send_cmd_none = True
        elif (True == self.send_cmd_none):
            self.cfg_pub.publish(self.cfg_cmd)
            self.send_cmd_none = False
        elif (False == self.send_cmd_none):
            if (joyMessage.axes[AXIS_L_TRIGGER] < -0.9):
                self.motion_cmd.linear.x = (joyMessage.axes[AXIS_UD_LSTICK] * self.vel_limit)
                self.motion_cmd.angular.z = (joyMessage.axes[AXIS_LR_RSTICK] * self.steer_limit)
                self.last_motion_command_time = rospy.Time.now().to_sec()
            else:
                self.motion_cmd.linear.x = 0.0
                self.motion_cmd.angular.z = 0.0
                
            if ((rospy.Time.now().to_sec() - self.last_motion_command_time) < 0.1):
                self.motion_pub.publish(self.motion_cmd)

    def xbox_joy_receive(self):
        rospy.Subscriber('/joy', Joy, self.xbox_to_rmp)
        print "xbox_to_rmp node started"
        rospy.spin()

if __name__ == "__main__":
    xbox_rmp = XBoxRMP()
    xbox_rmp.xbox_joy_receive()
    
