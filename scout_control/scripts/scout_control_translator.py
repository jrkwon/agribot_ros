#!/usr/bin/env python3

import rospy
from scout_msgs.msg import ScoutControl
from geometry_msgs.msg import Twist

import const
from config import Config

dc_config = Config.data_collection
rn_config = Config.run_neural

###################
# ScoutControl.msg
# ----------------
# uint8 NO_COMMAND=0
# uint8 NEUTRAL=1
# uint8 FORWARD=2
# uint8 REVERSE=3

GEARSHIFTS = {
    ScoutControl.NEUTRAL: "Neutral",
    ScoutControl.FORWARD: "Forward",
    ScoutControl.REVERSE: "Reverse"
}

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
        prefix = dc_config['teleop_twist_node_prefix']
        self.scale_linear = rospy.get_param(prefix + 'scale_linear', 0.4)
        self.scale_angular = rospy.get_param(prefix + 'scale_angular', 0.6)
        self.scale_linear_turbo = rospy.get_param(prefix + 'scale_linear_turbo', 1.0)
        self.scale_angular_turbo = rospy.get_param(prefix + 'scale_angular_turbo', 1.2)
        rospy.loginfo("scale_linear: %.2f, scale_angular: %.2f", 
                      self.scale_linear, self.scale_angular)
        rospy.loginfo("scale_linear_turbo: %.2f, scale_angular_turbo: %.2f", 
                      self.scale_linear_turbo, self.scale_angular_turbo)

    def _callback(self, message):
        rospy.logdebug("scout_control_translater received %s", message)

        twist_msg = Twist()

        # get scale values
        scale_linear  = self.scale_linear_turbo if message.enable_turbo else self.scale_linear
        scale_angular = self.scale_angular_turbo if message.enable_turbo else self.scale_angular
        
        if message.gearshift == ScoutControl.NEUTRAL:
            rospy.logwarn("Gearshift: %s.", GEARSHIFTS[message.gearshift])
            return
        
        if message.gearshift == ScoutControl.FORWARD:
            if message.throttle > 0:
                twist_msg.linear.x = message.throttle*scale_linear
            elif message.brake > 0:
                twist_msg.linear.x = -message.brake*scale_linear
        elif message.gearshift == ScoutControl.REVERSE:
            if message.throttle > 0:
                twist_msg.linear.x = -message.throttle*scale_linear
            elif message.brake > 0:
                twist_msg.linear.x = message.brake*scale_linear

        twist_msg.angular.z = message.steering*scale_angular

        rospy.loginfo("Turbo: %s, Gearshift: %s, Steering: %.2f, Throttle: %.2f.",
                      "Yes" if message.enable_turbo else "No",
                      GEARSHIFTS[message.gearshift], message.steering, message.throttle)

        self.last_published = twist_msg
        self.last_published_time = rospy.get_rostime()
        self.pub.publish(twist_msg)


if __name__ == '__main__':
    rospy.init_node('scout_control_translator')
    ScoutControlTranslator()
    rospy.spin()