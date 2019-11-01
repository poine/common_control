#!/usr/bin/env python
import sys, time, numpy as np
import rospy, geometry_msgs.msg

import two_d_guidance.trr.rospy_utils as trr_rpu
import pdb

class Node(trr_rpu.PeriodicNode):
    def __init__(self, ctl):
        trr_rpu.PeriodicNode.__init__(self, 'twist_drive_line')
        self.ctl = ctl
        cmd_topic = rospy.get_param('~cmd_topic', '/caroline/diff_drive_controller/cmd_vel')
        self.pub = rospy.Publisher(cmd_topic, geometry_msgs.msg.Twist, queue_size=1)
        odom_topic = rospy.get_param('~odom_topic', '/caroline/diff_drive_controller/odom')
        self.odom_sub = trr_rpu.OdomListener(odom_topic, 'odom calib')
       
    def publish_twist(self, lin, ang):
        msg =  geometry_msgs.msg.Twist()
        msg.linear.x, msg.angular.z = lin, ang
        #pdb.set_trace()
        self.pub.publish(msg)

    def periodic(self):
        try:
            p, psi = self.odom_sub.get_loc_and_yaw()
            if not self.ctl.initialized():
                self.ctl.initialize(p, psi)
            else:
                self.publish_twist(*self.ctl.get(p, psi))
                if self.ctl.finished(p, psi): self.finish()
        except trr_rpu.NoRXMsgException, trr_rpu.RXMsgTimeoutException:
            rospy.loginfo_throttle(1., "no odom")
        
    def finish(self):
        time.sleep(0.5)
        p, psi = self.odom_sub.get_loc_and_yaw()
        self.final_dist = self.ctl._dist(p)
        self.final_ang = self.ctl._ang(psi)
        #print('{}'.format(self.final_dist))
        #print('{}'.format(self.final_ang))
        rospy.signal_shutdown('done')
