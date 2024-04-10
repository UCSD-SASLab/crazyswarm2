#!/usr/bin/env python

from pathlib import Path

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    traj1 = Trajectory()
    traj1.loadcsv(Path(__file__).parent / 'data/figure8.csv')

    TRIALS = 1
    TIMESCALE = 1.0
    for i in range(TRIALS):
        allcfs.takeoff(targetHeight=1.0, duration=5.0)
        timeHelper.sleep(4.0)
        # allcfs.goTo([-4.5, 0.0, 1.0], 0.0, 5.)
        for cf in allcfs.crazyflies:
            cf.goTo([-4.5, 0.0, 1.0], 0.0, 5.)
            # cf.goTo([0.0, 0.0, 1.0], 0.0, 5.)
        timeHelper.sleep(5.) 
        j = 0
        while j < 1500:
            for cf in allcfs.crazyflies:
                ypos = np.sin(j / 100)
                pos=[-4.5, ypos, 1.0]
                # pos=[0., ypos, 1.0]
                vel=[0., 0., 0.]
                acc=[0., 0., 0.]
                yaw=0.
                omega=[0., 0., 0.]
                if j > 1250:
                    ypos = 0.0
                cf.cmdFullState(pos, vel, acc, yaw, omega)
            timeHelper.sleep(0.01)

            
            j += 1
        cf.notifySetpointsStop(10)
        for cf in allcfs.crazyflies:
            # cf.goTo([0., 0.0, 1.0], 0.0, 5.)
            cf.goTo([-4.5, 0.0, 1.0], 0.0, 5.)
        timeHelper.sleep(5) 
        # allcfs.startTrajectory(0, timescale=TIMESCALE, reverse=True)
        # timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)

        allcfs.land(targetHeight=0.06, duration=5.0)
        timeHelper.sleep(3.0)


if __name__ == '__main__':
    main()
