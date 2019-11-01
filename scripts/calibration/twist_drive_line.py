#!/usr/bin/env python
import sys, time, numpy as np
import rospy, ackermann_msgs.msg

''' 
Drive a line.
 still needs some love
'''

import two_d_guidance.trr.rospy_utils as trr_rpu
import twist_node

class Ctl:
    def __init__(self, _v, _d): self.v, self.d = _v, _d; self.p0=None
    def initialize(self, p0, psi0): self.p0, self.psi0 = p0, psi0
    def initialized(self): return self.p0 is not None
    def finished(self, p, psi): return self._dist(p) > self.d
    def get(self, p, psi): return (self.v, 0.) if not self.finished(p, psi) else (0., 0.)
    def _dist(self, p): return np.linalg.norm(p-self.p0)
    def _ang(self, psi): return psi-self.psi0
    
def main(args):
    node = twist_node.Node(Ctl(_v=0.2, _d=1.))
    truth_sub = trr_rpu.OdomListener('/caroline/base_link_truth', 'odom_calib')
    time.sleep(0.5)
    p0 = truth_sub.get_pose()
    node.run(50.)
    p1 = truth_sub.get_pose()
    d_truth = np.linalg.norm(p1-p0)
    d_odom = node.final_dist
    print('truth {:.3f}, odom {:.3f}, mult {:.4f}'.format(d_truth, d_odom, d_truth/d_odom))
    # 0.987 seems ok for caroline on gazebo

if __name__ == '__main__':
    main(sys.argv)
