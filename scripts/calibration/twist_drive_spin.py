#!/usr/bin/env python
import sys, time, numpy as np
import rospy, ackermann_msgs.msg

''' 
Spin on place.
'''

import two_d_guidance.trr.rospy_utils as trr_rpu
import two_d_guidance.utils as tdgu
import twist_node

class Ctl:
    def __init__(self, _v, _d): self.v, self.d = _v, _d; self.psi0=None
    def initialize(self, p0, psi0):
        self.p0, self.psi0 = p0, psi0
    def initialized(self): return self.psi0 is not None
    def finished(self, p, psi):
        d_psi = psi-self.psi0
        return tdgu.normalize_headings(d_psi) > np.pi/2
    def get(self, p, psi): return (0., self.v) if not self.finished(p, psi) else (0., 0.)
    def _dist(self, p): return np.linalg.norm(p-self.p0)
    def _ang(self, psi): return psi-self.psi0
            
def main(args):
    node = twist_node.Node(Ctl(_v=0.2, _d=1.))
    truth_sub = trr_rpu.OdomListener('/caroline/base_link_truth', 'odom_calib')
    time.sleep(0.5)
    l0, y0 = truth_sub.get_loc_and_yaw()
    node.run(50.)
    l1, y1 = truth_sub.get_loc_and_yaw()
    d_psi_truth = np.rad2deg(tdgu.normalize_headings(y1-y0))
    d_psi_odom = np.rad2deg(node.final_ang)
    print('truth {:.3f}, odom {:.3f}, mult {:.4f}'.format(d_psi_truth, d_psi_odom, d_psi_truth/d_psi_odom))

if __name__ == '__main__':
    main(sys.argv)
