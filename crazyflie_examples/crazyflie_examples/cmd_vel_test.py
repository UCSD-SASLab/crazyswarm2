#!/usr/bin/env python

from pathlib import Path

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np#!/usr/bin/env python

from pathlib import Path

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np


def main():

    xpos = -4.5
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=1.0, duration=5.0)
    timeHelper.sleep(4.0)
    for cf in allcfs.crazyflies:
        cf.goTo([xpos, 0.0, 1.0], 0.0, 5.)
    timeHelper.sleep(4.) 

 
    for j  in range(1256):
        for cf in allcfs.crazyflies:
            if j <= 5:
                cf.cmdVel(0., 0., 0., 0.)
            else:
                cf.cmdVel(0., 0., 0., 55000.)

            ## With cmdFullState (works)
            # pos = 1. + 0.5 * np.sin(j / 100)
            # pos = 0.5 * np.sin(j / 100)
            # cf.cmdFullState([xpos, pos, 1.], [0., 0., 0.], [0., 0., 0.], 0., [0., 0., 0.])

            ## With cmdVel (doesnt)
            # thrust = (j-50)*100.0
            # cf.cmdVel(0., 0., 0., thrust)
        
        timeHelper.sleep(0.01)

    ## Transfer back to HL
    cf.notifySetpointsStop(10)
    for cf in allcfs.crazyflies:
        cf.goTo([xpos, 0.0, 1.0], 0.0, 5.)
    timeHelper.sleep(5) 

    allcfs.land(targetHeight=0.06, duration=5.0)
    timeHelper.sleep(3.0)

if __name__ == '__main__':
    main()
