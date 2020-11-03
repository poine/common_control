import numpy as np
import rospy, geometry_msgs.msg, tf

class PeriodicNode:

    def __init__(self, name):
        rospy.init_node(name)
    
    def run(self, freq):
        rate = rospy.Rate(freq)
        try:
            while not rospy.is_shutdown():
                self.periodic()
                rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            pass

# class TwistNode(trr_rpu.PeriodicNode):
#     def __init__(self, ctl):
#         trr_rpu.PeriodicNode.__init__(self, 'twist_drive_line')
#         self.ctl = ctl
#         cmd_topic = rospy.get_param('~cmd_topic', '/caroline/diff_drive_controller/cmd_vel')
#         self.pub = rospy.Publisher(cmd_topic, geometry_msgs.msg.Twist, queue_size=1)
#         odom_topic = rospy.get_param('~odom_topic', '/caroline/diff_drive_controller/odom')
#         self.odom_sub = trr_rpu.OdomListener(odom_topic, 'odom calib')

#     def publish_twist(self, lin, ang):
#         msg =  geometry_msgs.msg.Twist()
#         msg.linear.x, msg.angular.z = lin, ang
#         #pdb.set_trace()
#         self.pub.publish(msg)

#     def periodic(self):
#         try:
#             p, psi = self.odom_sub.get_loc_and_yaw()
#             if not self.ctl.initialized():
#                 self.ctl.initialize(p, psi)
#             else:
#                 self.publish_twist(*self.ctl.get(p, psi))
#                 if self.ctl.finished(p, psi): self.finish()
#         except trr_rpu.NoRXMsgException, trr_rpu.RXMsgTimeoutException:
#             rospy.loginfo_throttle(1., "no odom")

#     def finish(self):
#         time.sleep(0.5)
#         p, psi = self.odom_sub.get_loc_and_yaw()
#         self.final_dist = self.ctl._dist(p)
#         self.final_ang = self.ctl._ang(psi)
#         #print('{}'.format(self.final_dist))
#         #print('{}'.format(self.final_ang))
#         rospy.signal_shutdown('done')

### originated from two_d_guidance/src/two_d_guidance/ros_utils.py
def list_of_xyz(p): return [p.x, p.y, p.z]
def array_of_xyz(p): return np.array(list_of_xyz(p))
def list_of_xyzw(q): return [q.x, q.y, q.z, q.w]

class RobotNotLocalizedException(Exception):
    pass
class RobotLostException(Exception):
    pass

class PoseListener:
    def __init__(self, topic='/drone/gt_pose'):
        self.pose = None
        self.ts = None
        rospy.Subscriber(topic, geometry_msgs.msg.Pose, self.msg_cbk)

    def msg_cbk(self, msg):
        self.pose = msg
        #self.ts = msg.header.stamp.to_sec()
        
    def get_loc_and_rpy(self, max_delay=0.2):
        if self.pose is None:
            raise RobotNotLocalizedException
        #if rospy.Time.now().to_sec() - self.ts > max_delay:
        #    raise RobotLostException
        l = array_of_xyz(self.pose.position)
        r = tf.transformations.euler_from_quaternion(list_of_xyzw(self.pose.orientation))
        return l, r

    def get_t_and_q(self, max_delay=0.2):
        if self.pose is None:
            raise RobotNotLocalizedException
        return array_of_xyz(self.pose.position), list_of_xyzw(self.pose.orientation)
