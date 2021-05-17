#!/usr/bin/env python3

import sys
import rospy
import rospkg
from panda_traj.srv import UpdateTrajectory, UpdateTrajectoryRequest


rospack = rospkg.RosPack()

def UpdateMyTrajectory(x, y):
    rospy.wait_for_service('/velocity_qp/updateTrajectory')
    try:
        client = rospy.ServiceProxy('/velocity_qp/updateTrajectory', UpdateTrajectory)

        req = UpdateTrajectoryRequest();
        req.csv_traj_path = x
        req.verbose = y
        client.call(req);
    except rospy.ServiceException as e:
        print ("Service call failed: ",e)

def usage():
    return "%s [traj_path verbose = (default false)]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        traj_path_ = rospack.get_path('panda_traj') + "/trajectories/" + str(sys.argv[1]) + ".csv"
        verbose_ = False
    elif len(sys.argv) == 3:
        traj_path_ = rospack.get_path('panda_traj') + "/trajectories/" + str(sys.argv[1])+".csv"
        verbose_ = bool(sys.argv[1])
    else:
        print(usage())
        sys.exit(1)
    
    print ("Requesting ",traj_path_)
    UpdateMyTrajectory(traj_path_, verbose_ )