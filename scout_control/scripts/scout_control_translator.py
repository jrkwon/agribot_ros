#!/usr/bin/env python3

import rospy
from scout_msgs.msg import ScoutControl
from geometry_msgs.msg import Twist

import const
from config import Config

dc_config = Config.data_collection
rn_config = Config.run_neural

##############################################################################
# Topic translator
# ============================================================================
# ScoutControl --> /joy_teleop/cmd_vel
#  
# Note: scaling factors similar to teleop_twist_joy must be applied.
# ----------------------------------------------------------------------------
class ScoutControlTranslator:
    def __init__(self):
        self.sub = rospy.Subscriber(dc_config['vehicle_control_topic'], 
                                    ScoutControl, self._callback)
        self.pub = rospy.Publisher(rn_config['cmd_vel_topic'], Twist, queue_size=1)
        self.last_published = None

    def _callback(self, message):
        rospy.logdebug("scout_control_translater received %s", message)

        # -------------------------------------------------------------------
        # teleop_twist_joy scaling
        # default values from teleop_logitech.yaml were used.
        scale_linear = rospy.get_param(
                        dc_config['teleop_twist_node_prefix']+'scale_angular', 0.4)
        scale_angular = rospy.get_param(
                        dc_config['teleop_twist_node_prefix']+'scale_linear', 0.6)

        twist_msg = Twist()

        if message.shift_gears == ScoutControl.NEUTRAL:
            return
        
        if message.shift_gears == ScoutControl.FORWARD:
            if message.throttle > 0:
                twist_msg.linear.x = message.throttle*scale_linear
            elif message.brake > 0:
                twist_msg.linear.x = -message.brake*scale_linear
        elif message.shift_gears == ScoutControl.BACKWARD:
            if message.throttle > 0:
                twist_msg.linear.x = -message.throttle*scale_linear
            elif message.brake > 0:
                twist_msg.linear.x = message.brake*scale_linear

        twist_msg.angular.z = message.steering*scale_angular

        print(f'Message received: steering: {message.steering}, throttle: {message.throttle}')

        self.last_published = twist_msg
        self.last_published_time = rospy.get_rostime()
        self.pub.publish(twist_msg)


if __name__ == '__main__':
    rospy.init_node('scout_control_translator')
    ScoutControlTranslator()
    rospy.spin()