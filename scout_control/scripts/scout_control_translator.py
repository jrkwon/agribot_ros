#!/usr/bin/env python3

import rospy
from scout_msgs.msg import ScoutControl
from geometry_msgs.msg import Twist

import const
from config import Config

dc_config = Config.data_collection
rn_config = Config.run_neural

######################################
# Topic translator
# ------------------------------------
# ScoutControl --> /joy_teleop/cmd_vel
#  
class ScoutControlTranslator:
    def __init__(self):
        # self.sub = rospy.Subscriber(dc_config['vehicle_control_topic'], ScoutControl, self.callback)
        # self.pub = rospy.Publisher(rn_config['cmd_vel_topic'], Twist, queue_size=1)

        # self.last_published_time = rospy.get_rostime()
        self.last_published = None
        # self.timer = rospy.Timer(rospy.Duration(1./20.), self.timer_callback)
        
    # def _timer_callback(self, event):
    #     if self.last_published and self.last_published_time < rospy.get_rostime() + rospy.Duration(1.0/20.):
    #         self._callback(self.last_published)

    def _callback(self, message):
        rospy.logdebug("scout_control_translater received %s", message)
        
        twist_msg = Twist()
        twist_msg.linear.x = message.throttle
        twist_msg.angular.z = message.steering
        print(f'Message received: steering: {message.steering}, throttle: {message.throttle}')

        self.last_published = twist_msg
        self.last_published_time = rospy.get_rostime()
        self.pub.publish(twist_msg)

    def start(self):
        self.sub = rospy.Subscriber(dc_config['vehicle_control_topic'], ScoutControl, self._callback)
        self.pub = rospy.Publisher(rn_config['cmd_vel_topic'], Twist, queue_size=1)

        # self.timer = rospy.Timer(rospy.Duration(1./20.), self._timer_callback)


if __name__ == '__main__':
    rospy.init_node('scout_control_translator')
    t = ScoutControlTranslator()
    t.start()
    rospy.spin()