#!/usr/bin/env python
from __future__ import division, print_function

import sys

import kdl_parser_py.urdf
import unittest
import PyKDL as kdl
import rostest
import datetime
def main():
    filename = None
    if (sys.argv > 1):
        filename = 'uthai_description/urdf/uthai.urdf'
    (ok, tree) = kdl_parser_py.urdf.treeFromFile(filename)

    chain = tree.getChain("base_footprint","r_foot_ft_link")
    fksolver = kdl.ChainFkSolverPos_recursive(chain)
    ikvelsolver = kdl.ChainIkSolverVel_pinv(chain)
    iksolver = kdl.ChainIkSolverPos_NR(chain,fksolver,ikvelsolver)

    q_init = kdl.JntArray(6)
    q = kdl.JntArray(6)
    F_dest = kdl.Frame(kdl.Rotation.RPY(0,0,0),kdl.Vector(0.0,0,0))

    status = iksolver.CartToJnt(q_init,F_dest,q)
    print("Status : ",status)
    print(q)


if __name__ == '__main__':
    main()