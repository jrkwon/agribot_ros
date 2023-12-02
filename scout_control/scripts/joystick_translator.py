#!/usr/bin/env python3

# Copyright 2017 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rospy
from scout_msgs.msg import ScoutControl
from sensor_msgs.msg import Joy

import const
from config import Config

config = Config.data_collection

#######################################
## Logitech F710 

######################
# teleop_logitech.yaml
# --------------------
# Teleop configuration for Logitech F710 Gamepad using the x-pad configuration.
# Original User Interface Design in "teleop_twist_joy"
#   Left thumb-stick up/down for velocity, left/right for twist
#   LB for enable
#   RB for enable-turbo
# New Design for Agribot
#   Left thumb-stick up/down for velocity, right thumb-stick left/right for twist
#   LB for enable
#   RB for enable-turbo
#   Gearshift:
#       Y: Forward
#       A: Reverse
#       B: Neutral
#
#         (LB)                                 (RB)
#         (LT)                                 (RT)
#       _=====_            D(  .)X            _=====_
#      / _____ \                             / _____ \
#    +.-'_____'-.---------------------------.-'_____'-.+
#   /   |     |  '.                       .'  |      |   \
#  / ___| /|\ |___ \ (back)(Lgtc)(strt)  / ___| (Y)  |___ \
# / |      |      | ;  __           __  ; |              | |
# | | <---   ---> | | (__) .       (__) | | (X)       (B)| |
# | |___   |   ___| ; MODE         VIBE ; |___       ____| /
# |\    | \|/ |    /  _     ___      _   \    | (A) |    /|
# | \   |_____|  .','" "', |___|  ,'" "', '.  |_____|  .' |
# |  '-.______.-' /       \ANALOG/       \  '-._____.-'   |
# |               |  LJ   |------|   RJ  |                |
# |              /\       /      \       /\               |
# |             /  '.___.'        '.___.'  \              |
# |            /                            \             |
#  \          /                              \           /
#   \________/                                \_________/
## --------------------
# BUTTON         Value
#   LB             4
#   RB             5
#   A              0
#   B              1
#   X              2
#   Y              3
#
#    AXIS        Value
# Left Horiz.      0
# Left Vert.       1
# Right Horiz.     3
# Right Vert.      4
# Left Trigger     2
# Right Trigger    5
# D-pad Horiz.     6
# D-pad Vert.      7

################################
# Definition of Buttons and Axes
BUTTON_A        = 0        
BUTTON_B        = 1        
BUTTON_X        = 2        
BUTTON_Y        = 3       
BUTTON_LB       = 4
BUTTON_RB       = 5

LEFT_HORIZ      = 0
LEFT_VERT       = 1
RIGHT_HORIZ     = 3
RIGHT_VERT      = 4
LEFT_TRIGGER    = 2
RIGHT_TRIGGER   = 5
D_PAD_HORIZ     = 6
D_PAD_VERT      = 7


#####################
# Steering
STEERING_AXIS = RIGHT_HORIZ   # left 1 --> center 0 --> right -1

#####################
# Enables
ENABLE = BUTTON_LB
ENABLE_TURBO = BUTTON_RB

#####################
# Throttle and Brake: 
# NOTE for Brake
# --------------
# No brake is for the Scout robot. 
# When BRAKE_AXIS is down with the FORWARD gearshift, 
# the robot actually moves backward.
THROTTLE_AXIS   = LEFT_VERT   # up from middle   (0 ~ 1)   
BRAKE_AXIS      = LEFT_VERT      # down from middle (0 ~ -1)
BRAKE_POINT     = -0.2  # consider brake is applied if value is greater than this.

# Gear shift
# to be neutral, bothe SHIFT_FORWARD & SHIFT_REVERSE must be 0
SHIFT_FORWARD   = BUTTON_Y     # forward 1
SHIFT_REVERSE   = BUTTON_A     # reverse 1
SHIFT_NEUTRAL   = BUTTON_B     # neutral 1

# Small value
SMALL_VALUE     = 0.0001


class JoystickTranslator:
    def __init__(self):
        self.sub = rospy.Subscriber(config['joystick_topic'], Joy, self.callback)
        self.pub = rospy.Publisher(config['vehicle_control_topic'], ScoutControl, queue_size=1)
        self.last_published_time = rospy.get_rostime()
        self.last_published = None
        self.timer = rospy.Timer(rospy.Duration(1./20.), self.timer_callback)
        self.control = ScoutControl()
        self.control.gearshift = ScoutControl.NEUTRAL
        self.no_joy_msg_printed = False


    def timer_callback(self, event):
        if self.last_published and self.last_published_time < rospy.get_rostime() + rospy.Duration(1.0/20.):
            self.callback(self.last_published)


    def callback(self, message):
        rospy.logdebug("joy_translater received axes %s", message.axes)
        if (len(message.buttons) <= 5): # not a real F710 joystick. abort...
            if self.no_joy_msg_printed is False:
                rospy.logwarn("No proper joystick is attached.")
                self.no_joy_msg_printed = True
            return

        self.control.header = message.header

        ##############################
        # Check Enable or Enalbe-Turbo 
        enable = self.control.enable_turbo = False
        # If both ENABLE and ENABLE_TURBO buttons are pressed at the same time
        # Consider only ENABLE
        if message.buttons[ENABLE_TURBO] == 1:
            self.control.enable_turbo = True
        if message.buttons[ENABLE] == 1:
            self.control.enable_turbo = False
            enable = True

        if enable is False and self.control.enable_turbo is False:
            rospy.loginfo("Either Enable/Enable-Turbo button should be pressed.")
            return

        if message.axes[BRAKE_AXIS] < BRAKE_POINT:
            self.control.brake = -message.axes[BRAKE_AXIS]

        # Note: init value of axes are all zeros
        # --> problem with -1 to 1 range values like brake
        if message.axes[BRAKE_AXIS] > -1*SMALL_VALUE and message.axes[BRAKE_AXIS] < SMALL_VALUE:
            self.control.brake = 0.0

        if message.axes[THROTTLE_AXIS] >= 0:
            self.control.throttle = message.axes[THROTTLE_AXIS]
            self.control.brake = 0.0
        else: # braking
            self.control.throttle = 0.0

        if message.buttons[SHIFT_FORWARD] == 1:
            self.control.gearshift = ScoutControl.FORWARD
            rospy.loginfo("Gearshift: FORWARD")
        elif message.buttons[SHIFT_REVERSE] == 1:
            self.control.gearshift = ScoutControl.REVERSE
            rospy.loginfo("Gearshift: REVERSE")
        elif message.buttons[SHIFT_NEUTRAL] == 1:
            self.control.gearshift = ScoutControl.NEUTRAL
            rospy.loginfo("Gearshift: NEUTRAL")

        self.control.steering = message.axes[STEERING_AXIS]
        self.last_published = message
        self.pub.publish(self.control)


if __name__ == '__main__':
    rospy.init_node('joystick_translator')
    JoystickTranslator()
    rospy.spin()