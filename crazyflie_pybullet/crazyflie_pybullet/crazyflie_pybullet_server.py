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


# import BackendRviz from .backend_rviz
# from .backend import *
# from .backend.none import BackendNone
from .crazyflie_sil import CrazyflieSIL, TrajectoryPolynomialPiece
from .sim_data_types import State

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

        # Turn ROS parameters into a dictionary
        self._ros_parameters = self._param_to_dict(self._parameters)
        self.cfs = {}
        
        names = ["cf231"]
        initial_states = [State(np.array([0,0,0]))]
        controller_name = 'mellinger'

        # create robot SIL objects
        for name, initial_state in zip(names, initial_states):
            self.cfs[name] = CrazyflieSIL(
                name,
                initial_state.pos,
                controller_name,
                self._time
                )
            
        self.pose_publishers = dict()
        self.odom_publishers = dict()
        for name, _ in self.cfs.items():
            # pub = self.create_publisher(
            #         String,
            #         name + '/robot_description',
            #         rclpy.qos.QoSProfile(
            #             depth=1,
            #             durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL))

            # msg = String()
            # msg.data = self._ros_parameters['robot_description'].replace('$NAME', name)
            # pub.publish(msg)

            self.create_service(
                Empty,
                name + '/emergency',
                partial(self._emergency_callback, name=name)
            )
            self.create_service(
                Takeoff,
                name + '/takeoff',
                partial(self._takeoff_callback, name=name)
            )
            self.create_service(
                Land,
                name + '/land',
                partial(self._land_callback, name=name)
            )
            self.create_service(
                GoTo,
                name + '/go_to',
                partial(self._go_to_callback, name=name)
            )
            self.create_service(
                StartTrajectory,
                name + '/start_trajectory',
                partial(self._start_trajectory_callback, name=name)
            )
            self.create_service(
                UploadTrajectory,
                name + '/upload_trajectory',
                partial(self._upload_trajectory_callback, name=name)
            )
            self.create_service(
                NotifySetpointsStop,
                name + '/notify_setpoints_stop',
                partial(self._notify_setpoints_stop_callback, name=name)
            )
            self.create_subscription(
                Twist,
                name + '/cmd_vel_legacy',
                partial(self._cmd_vel_legacy_changed, name=name),
                10
            )
            self.create_subscription(
                Hover,
                name + '/cmd_hover',
                partial(self._cmd_hover_changed, name=name),
                10
            )
            self.create_subscription(
                FullState,
                name + '/cmd_full_state',
                partial(self._cmd_full_state_changed, name=name),
                10
            )

            self.create_subscription(
                FullState,
                name + '/disturbance',
                partial(self._cmd_disturbance_changed, name=name),
                10
            )

            self.pose_publishers[name] = self.create_publisher(PoseStamped, name + "/pose", 10)
            self.odom_publishers[name] = self.create_publisher(Odometry, name + "/odom", 10)

        # Create services for the entire swarm and each individual crazyflie
        self.create_service(Takeoff, 'all/takeoff', self._takeoff_callback)
        self.create_service(Land, 'all/land', self._land_callback)
        self.create_service(GoTo, 'all/go_to', self._go_to_callback)
        self.create_service(StartTrajectory,
                            'all/start_trajectory',
                            self._start_trajectory_callback)

        # This is the last service to announce.
        # Can be used to check if the server is fully available.
        self.create_service(Empty, 'all/emergency', self._emergency_callback)

        # step as fast as possible
        # max_dt = 0.0 if 'max_dt' not in self._ros_parameters['sim'] \
        #     else self._ros_parameters['sim']['max_dt']
        
        # Set timestep to control frequency
        self.max_dt = 1./self.env.ctrl_freq

        self.timer = self.create_timer(self.max_dt, self._timer_callback)
        self.is_shutdown = False

        # Track start time for simulation sync
        self.start_time = time.time()

        # Simulation control index for PyBullet
        self.i = 0

        # Keeps track fo simulation time
        self.t = 0

    def on_shutdown_callback(self):
        if not self.is_shutdown:
            # REPLACE WITH PYBULLET
            # self.backend.shutdown()
            # for visualization in self.visualizations:
            #     visualization.shutdown()
            self.env.close()
            self.pyb_logger.save()
            self.pyb_logger.save_as_csv("beta")
            self.is_shutdown = True

    def _timer_callback(self):
        # self.t = (time.time() - self.start_time) % 60
        self.t += self.max_dt

        # update setpoint
        states_desired = [cf.getSetpoint() for _, cf in self.cfs.items()]
        # execute the control loop
        actions = [cf.executeController() for _, cf in self.cfs.items()]
        disturbances = [cf.getDisturbance() for _, cf in self.cfs.items()]

        print(states_desired, actions, disturbances)
        # execute the physics simulator
        # REPLACE WITH PYBULLET
        # states_next = self.backend.step(states_desired, actions, disturbances)
        obs, reward, terminated, truncated, info = self.env.step(self.i)

        # Step Forward in PyBullet
        # for vis in self.visualizations:
        #     vis.step(self.backend.time(), states_next, states_desired, actions)  
        self.env.render()

        # Sync simulation
        sync(self.i, self.start_time, self.env.CTRL_TIMESTEP) 

        # Advance simulation control index by one
        self.i += 1      

    def _log_pose_data_callback(self, name, state):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_tf_name

        msg.pose.position.x = state.pos[0]
        msg.pose.position.y = state.pos[1]
        msg.pose.position.z = state.pos[2]

        msg.pose.orientation.w = state.quat[0]
        msg.pose.orientation.x = state.quat[1]
        msg.pose.orientation.y = state.quat[2]
        msg.pose.orientation.z = state.quat[3]
        self.pose_publishers[name].publish(msg)

    def _log_odom_data_callback(self, name, state):
        msg = Odometry()
        msg.child_frame_id = name
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_tf_name

        msg.pose.pose.position.x = state.pos[0]
        msg.pose.pose.position.y = state.pos[1]
        msg.pose.pose.position.z = state.pos[2]

        msg.pose.pose.orientation.w = state.quat[0]
        msg.pose.pose.orientation.x = state.quat[1]
        msg.pose.pose.orientation.y = state.quat[2]
        msg.pose.pose.orientation.z = state.quat[3]

        msg.twist.twist.linear.x = state.vel[0]
        msg.twist.twist.linear.y = state.vel[1]
        msg.twist.twist.linear.z = state.vel[2]

        msg.twist.twist.angular.x = state.omega[0]
        msg.twist.twist.angular.y = state.omega[1]
        msg.twist.twist.angular.z = state.omega[2]

        self.odom_publishers[name].publish(msg)

    def _param_to_dict(self, param_ros):
        """Turn ROS 2 parameters from the node into a dict."""
        tree = {}
        for item in param_ros:
            t = tree
            for part in item.split('.'):
                if part == item.split('.')[-1]:
                    t = t.setdefault(part, param_ros[item].value)
                else:
                    t = t.setdefault(part, {})
        return tree

    def _emergency_callback(self, request, response, name='all'):
        self.get_logger().info(f'[{name}] emergency not yet implemented')

        return response

    def _takeoff_callback(self, request, response, name='all'):
        """Service callback to takeoff the crazyflie."""
        duration = float(request.duration.sec) + \
            float(request.duration.nanosec / 1e9)
        self.get_logger().info(
            f'[{name}] takeoff(height={request.height} m,'
            + f'duration={duration} s,'
            + f'group_mask={request.group_mask})'
        )
        cfs = self.cfs if name == 'all' else {name: self.cfs[name]}
        for _, cf in cfs.items():
            cf.takeoff(request.height, duration, request.group_mask)

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
        cfs = self.cfs if name == 'all' else {name: self.cfs[name]}
        for _, cf in cfs.items():
            cf.land(request.height, duration, request.group_mask)

        return response

    def _go_to_callback(self, request, response, name='all'):
        """Service callback to have the crazyflie go to a position."""
        duration = float(request.duration.sec) + \
            float(request.duration.nanosec / 1e9)

        self.get_logger().info(
            """[%s] go_to(position=%f,%f,%f m,
             yaw=%f rad,
             duration=%f s,
             relative=%d,
             group_mask=%d)"""
            % (
                name,
                request.goal.x,
                request.goal.y,
                request.goal.z,
                request.yaw,
                duration,
                request.relative,
                request.group_mask,
            )
        )
        cfs = self.cfs if name == 'all' else {name: self.cfs[name]}
        for _, cf in cfs.items():
            cf.goTo([request.goal.x, request.goal.y, request.goal.z],
                    request.yaw, duration, request.relative, request.group_mask)

        return response

    def _notify_setpoints_stop_callback(self, request, response, name='all'):
        self.get_logger().info(f'[{name}] Notify setpoint stop not yet implemented. Should not impact performance')
        return response

    def _upload_trajectory_callback(self, request, response, name='all'):
        self.get_logger().info('[%s] Upload trajectory(id=%d)' % (name, request.trajectory_id))

        cfs = self.cfs if name == 'all' else {name: self.cfs[name]}
        for _, cf in cfs.items():
            pieces = []
            for piece in request.pieces:
                poly_x = piece.poly_x
                poly_y = piece.poly_y
                poly_z = piece.poly_z
                poly_yaw = piece.poly_yaw
                duration = float(piece.duration.sec) + \
                    float(piece.duration.nanosec / 1e9)
                pieces.append(TrajectoryPolynomialPiece(
                    poly_x,
                    poly_y,
                    poly_z,
                    poly_yaw,
                    duration))
            cf.uploadTrajectory(request.trajectory_id, request.piece_offset, pieces)

        return response

    def _start_trajectory_callback(self, request, response, name='all'):
        self.get_logger().info(
            '[%s] start_trajectory(id=%d, timescale=%f, reverse=%d, relative=%d, group_mask=%d)'
            % (
                name,
                request.trajectory_id,
                request.timescale,
                request.reversed,
                request.relative,
                request.group_mask,
            )
        )
        cfs = self.cfs if name == 'all' else {name: self.cfs[name]}
        for _, cf in cfs.items():
            cf.startTrajectory(
                request.trajectory_id,
                request.timescale,
                request.reversed,
                request.relative,
                request.group_mask)

        return response

    def _cmd_vel_legacy_changed(self, msg, name=""):
        """
        Topic update callback to control the attitude and thrust
            of the crazyflie with teleop
        """
        roll = msg.linear.y
        pitch = -msg.linear.x
        yawrate = msg.angular.z
        thrust = int(min(max(msg.linear.z, 0, 0), 65535))

        self.get_logger().info('cmdvel: (%f, %f, %f, %d) ' % (roll, pitch, yawrate, thrust), throttle_duration_sec=5.0)

        self.cfs[name].cmdVelLegacy(roll, pitch, yawrate, thrust)
        
    def _cmd_hover_changed(self, msg, name=''):
        """
        Topic update callback for hover command.

        Used from the velocity multiplexer (vel_mux).
        """
        self.get_logger().info('cmd_hover not yet implemented')

    def _cmd_full_state_changed(self, msg, name):
        q = [msg.pose.orientation.w,
             msg.pose.orientation.x,
             msg.pose.orientation.y,
             msg.pose.orientation.z]
        rpy = rowan.to_euler(q, convention='xyz')

        self.cfs[name].cmdFullState(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z],
            [msg.acc.x, msg.acc.y, msg.acc.z],
            rpy[2],
            [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])

    def _cmd_disturbance_changed(self, msg, name):
        self.cfs[name].setDisturbance(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
        )

    def _time(self):
        return self.t


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
