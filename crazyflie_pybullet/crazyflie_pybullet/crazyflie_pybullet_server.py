#!/usr/bin/env python3

"""
A PyBullet server for crazyflie simulations.

    2025 - Kaleb Ugalde (UC San Diego)
"""

import time
from functools import partial
import importlib
import numpy as np

from crazyflie_interfaces.msg import FullState, Hover
from crazyflie_interfaces.srv import GoTo, Land, Takeoff
from crazyflie_interfaces.srv import NotifySetpointsStop, StartTrajectory, UploadTrajectory
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
import rowan
from std_msgs.msg import String
from std_srvs.srv import Empty

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CFAviary import CFAviary
from gym_pybullet_drones.control.CTBRControl import CTBRControl
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync

DEFAULT_DRONES = DroneModel("cf2x")
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_GUI = True
DEFAULT_PLOT = True
DEFAULT_USER_DEBUG_GUI = False
DEFAULT_SIMULATION_FREQ_HZ = 500
DEFAULT_CONTROL_FREQ_HZ = 25
DEFAULT_OUTPUT_FOLDER = 'results'
NUM_DRONES = 1
INIT_XYZ = np.array([[.5*i, .5*i, .1] for i in range(NUM_DRONES)])
INIT_RPY = np.array([[.0, .0, .0] for _ in range(NUM_DRONES)])

class CrazyfliePyBulletServer(Node):

    def __init__(self):
        super().__init__(
            'crazyflie_pybullet_server',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        # Create PyBullet Environment Object
        self.env = CFAviary(drone_model=DEFAULT_DRONES,
                            num_drones=NUM_DRONES,
                            gui=DEFAULT_GUI,
                            user_debug_gui=DEFAULT_USER_DEBUG_GUI,
                            pyb_freq=DEFAULT_SIMULATION_FREQ_HZ,
                            ctrl_freq=DEFAULT_CONTROL_FREQ_HZ)
        self.pyb_client = self.env.getPyBulletClient()

        # Create logger for PyBullet
        self.pyb_logger = Logger(logging_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
                                 num_drones=NUM_DRONES,
                                 output_folder=DEFAULT_OUTPUT_FOLDER)
        
        self.create_service(
            Takeoff,
            'cf231/takeoff',
            partial(self._takeoff_callback, name="cf231")
        )

        self.create_service(
            Land,
            'cf231/land',
            partial(self._land_callback, name="cf231")
        )

        # Set timestep to control frequency
        self.max_dt = 1./self.env.ctrl_freq

        self.timer = self.create_timer(self.max_dt, self._timer_callback)
        self.is_shutdown = False

        # Track start time for simulation sync
        self.start_time = time.time()

        # Simulation control index for PyBullet
        self.i = 0

    def on_shutdown_callback(self):
        if not self.is_shutdown:
            self.env.close()
            self.pyb_logger.save()
            self.pyb_logger.save_as_csv("beta")
            self.is_shutdown = True

    def _timer_callback(self):
        t = self.i/self.env.ctrl_freq

        # Step Simulation
        obs, reward, terminated, truncated, info = self.env.step(self.i)
        
        # Receive desired position
        pos = [0,0,0]
        vel = np.zeros(3)
        acc = np.zeros(3)
        yaw = 0
        rpy_rate = np.zeros(3)
        # self.env.sendFullStateCmd(pos, vel, acc, yaw, rpy_rate, t)

        # self.env.render()

        # Sync simulation
        sync(self.i, self.start_time, self.env.CTRL_TIMESTEP) 

        # Advance simulation control index by one
        self.i += 1

    def _takeoff_callback(self, request, response, name='all'):
        """Service callback to takeoff the crazyflie."""
        duration = float(request.duration.sec) + \
            float(request.duration.nanosec / 1e9)
        self.get_logger().info(
            f'[{name}] takeoff(height={request.height} m,'
            + f'duration={duration} s,'
            + f'group_mask={request.group_mask})'
        )
        
        self.env.sendTakeoffCmd(request.height, duration)

        return response

    def _land_callback(self, request, response, name='all'):
        """Service callback to land the crazyflie."""
        duration = float(request.duration.sec) + \
            float(request.duration.nanosec / 1e9)
        self.get_logger().info(
            f'[{name}] land(height={request.height} m,'
            + f'duration={duration} s,'
            + f'group_mask={request.group_mask})'
        )
        
        self.env.sendLandCmd(request.height, duration)

        return response      

def main(args=None):

    rclpy.init(args=args)
    crazyflie_server = CrazyfliePyBulletServer()
    rclpy.get_default_context().on_shutdown(crazyflie_server.on_shutdown_callback)

    try:
        rclpy.spin(crazyflie_server)
    except KeyboardInterrupt:
        crazyflie_server.on_shutdown_callback()
    finally:
        rclpy.try_shutdown()
        crazyflie_server.destroy_node()


if __name__ == '__main__':
    main()
