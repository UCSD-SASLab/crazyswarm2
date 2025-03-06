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
from example_interfaces.msg import Float64MultiArray

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync

from .backend.SASAviary import SASAviary

DEFAULT_DRONES = DroneModel("cf2x")
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_GUI = True
DEFAULT_PLOT = True
DEFAULT_RECORD_VIDEO = False
DEFAULT_OBSTACLES = False
DEFAULT_USER_DEBUG_GUI = False
DEFAULT_SIMULATION_FREQ_HZ = 500
DEFAULT_CONTROL_FREQ_HZ = 25
DEFAULT_OUTPUT_FOLDER = 'results'
DEFAULT_USE_FULL_STATE_CMD = False
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
        self.env = SASAviary(drone_model=DEFAULT_DRONES,
                             num_drones=NUM_DRONES,
                             gui=DEFAULT_GUI,
                             user_debug_gui=DEFAULT_USER_DEBUG_GUI,
                             pyb_freq=DEFAULT_SIMULATION_FREQ_HZ,
                             ctrl_freq=DEFAULT_CONTROL_FREQ_HZ,
                             use_full_state_cmd=DEFAULT_USE_FULL_STATE_CMD)
        self.use_full_state_cmd = DEFAULT_USE_FULL_STATE_CMD
        # self.env = VelocityAviary(drone_model=DEFAULT_DRONES,
        #                           num_drones=1,
        #                           initial_xyzs=INIT_XYZ,
        #                           initial_rpys=INIT_RPY,
        #                           physics=DEFAULT_PHYSICS,
        #                           neighbourhood_radius=10,
        #                           pyb_freq=DEFAULT_SIMULATION_FREQ_HZ,
        #                           ctrl_freq=DEFAULT_CONTROL_FREQ_HZ,
        #                           gui=DEFAULT_GUI,
        #                           record=DEFAULT_RECORD_VIDEO,
        #                           obstacles=DEFAULT_OBSTACLES,
        #                           user_debug_gui=DEFAULT_USER_DEBUG_GUI
        #                          )
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

        self.create_service(
            NotifySetpointsStop,
            'cf231/notify_setpoints_stop',
            partial(self._notify_setpoints_stop_callback, name='cf231')
        )

        self.create_subscription(
            Float64MultiArray,
            'cf_interface/control',
            partial(self._update_control, name='cf231'),
            10
        )

        self.create_subscription(
            Float64MultiArray,
            'cf231/target',
            partial(self._target_position_callback, name='cf231'),
            10
        )

        self.state_publisher = self.create_publisher(
            Float64MultiArray,
            "cf_interface/state",
            1
        )

        # Set timestep to control frequency
        self.max_dt = 1./self.env.ctrl_freq

        self.timer = self.create_timer(self.max_dt, self._timer_callback)
        self.is_shutdown = False

        # Track start time for simulation sync
        self.start_time = time.time()

        # Simulation control index for PyBullet
        self.i = 0

        self.goal_position = [0, 0, 0]

        self.thrust = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

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
        state_msg = Float64MultiArray()
        state = np.concatenate((self.env.pos[0], self.env.vel[0], self.env.quat[0]))
        state_msg.data = list(state)

        self.state_publisher.publish(state_msg)

        if self.use_full_state_cmd:
            self.env.sendFullStateCmd(pos=self.goal_position,
                                      vel=np.zeros(3),
                                      acc=np.zeros(3),
                                      yaw=0,
                                      rpy_rate=np.zeros(3),
                                      timestep=t)
        else:
            self.env.sendThrustRPYCmd(self.thrust,
                                      self.roll,
                                      self.pitch,
                                      self.yaw)

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

    def _notify_setpoints_stop_callback(self, request, response, name='all'):
        self.get_logger().info(f'[{name}] Notify setpoint stop not yet implemented. Should not impact performance')
        return response

    def _update_control(self, msg, name=""):
        """
        Topic update callback to control the attitude and thrust
            of the crazyflie with teleop
        """
        roll, pitch, yaw, thrust = msg.data
        self.thrust = thrust
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def _target_position_callback(self, msg, name=""):
        self.goal_position = msg.data

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
