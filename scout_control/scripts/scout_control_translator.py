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
        # -------------------------------------------------------------------
        # teleop_twist_joy scaling
        # default values from teleop_logitech.yaml were used.
        rospy.loginfo("param name: %s", dc_config['teleop_twist_node_prefix']+'scale_angular')
        self.scale_linear = rospy.get_param(
                        dc_config['teleop_twist_node_prefix']+'scale_linear', 0.4)
        self.scale_angular = rospy.get_param(
                        dc_config['teleop_twist_node_prefix']+'scale_angular', 0.6)
        rospy.loginfo("scale_linear: %.2f, scale_angular: %.2f", 
                      self.scale_linear, self.scale_angular)

    def _callback(self, message):
        rospy.logdebug("scout_control_translater received %s", message)


        twist_msg = Twist()

        if message.gearshift == ScoutControl.NEUTRAL:
            rospy.loginfo("Message received: gearshift is NEUTRAL (%d).", message.gearshift)
            return
        
        if message.gearshift == ScoutControl.FORWARD:
            if message.throttle > 0:
                twist_msg.linear.x = message.throttle*self.scale_linear
            elif message.brake > 0:
                twist_msg.linear.x = -message.brake*self.scale_linear
        elif message.gearshift == ScoutControl.REVERSE:
            if message.throttle > 0:
                twist_msg.linear.x = -message.throttle*self.scale_linear
            elif message.brake > 0:
                twist_msg.linear.x = message.brake*self.scale_linear

        twist_msg.angular.z = message.steering*self.scale_angular

        rospy.loginfo("Message received: gearshift: %d, steering: %2.f, throttle: %.2f.",
                      message.gearshift, message.steering, message.throttle)

        self.last_published = twist_msg
        self.last_published_time = rospy.get_rostime()
        self.pub.publish(twist_msg)


if __name__ == '__main__':
    rospy.init_node('scout_control_translator')
    ScoutControlTranslator()
    rospy.spin()