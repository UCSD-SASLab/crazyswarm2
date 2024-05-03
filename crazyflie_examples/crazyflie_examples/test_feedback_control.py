#!/usr/bin/env python
from crazyflie_py import FeedbackController, FeedbackController_Fig8, FeedbackController_hopf
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


def main():
    Z = 1.0

    # swarm = FeedbackController_Fig8()
    swarm = FeedbackController_hopf()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
        cf.goTo(pos, 0, 5.0)

    print('press button to start custom controller...')
    swarm.input.waitUntilButtonPressed()

    if not rclpy.ok():
        rclpy.init()
    node = Node('start_pub')
    publisher = node.create_publisher(String, 'start_custom_controller', 10)
    publisher.publish(String(data="start"))

    print('press button to land...')
    swarm.input.waitUntilButtonPressed()

    publisher.publish(String(data="stop"))
    allcfs.land(targetHeight=0.02, duration=1.0+Z)
    timeHelper.sleep(1.0+Z)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
