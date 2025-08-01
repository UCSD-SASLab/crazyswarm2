#!/usr/bin/env python3

"""
A crazyflie server for simulation.

    2022 - Wolfgang Hönig (TU Berlin)
    2025 - Updated by Kimberly N. McGuire (Independent)
"""

from functools import partial
import importlib

from crazyflie_interfaces.msg import FullState, Hover
from crazyflie_interfaces.srv import GoTo, Land, Takeoff
from crazyflie_interfaces.srv import NotifySetpointsStop, StartTrajectory, UploadTrajectory
from crazyflie_interfaces.srv import Arm
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
from rclpy.time import Time


class CrazyflieServer(Node):

    def __init__(self):
        super().__init__(
            'crazyflie_server',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        # Turn ROS parameters into a dictionary
        self._ros_parameters = self._param_to_dict(self._parameters)
        self.cfs = {}
        self.world_tf_name = 'world'
        robot_yaml_version = 0

        try:
            robot_yaml_version = self._ros_parameters['fileversion']
        except KeyError:
            self.get_logger().info('No fileversion found in crazyflies.yaml, assuming version 0')

        robot_data = self._ros_parameters['robots']

        # Parse robots
        names = []
        initial_states = []
        reference_frames = []
        for cfname in robot_data:
            if robot_data[cfname]['enabled']:
                type_cf = robot_data[cfname]['type']
                # do not include virtual objects
                connection = self._ros_parameters['robot_types'][type_cf].get(
                    'connection', 'crazyflie')
                if connection == 'crazyflie':
                    names.append(cfname)
                    pos = robot_data[cfname]['initial_position']
                    initial_states.append(State(pos))
                    # Get the current reference frame for the robot
                    reference_frame = self.world_tf_name
                    if robot_yaml_version >= 3:
                        try:
                            reference_frame = self._ros_parameters['all']['reference_frame']
                        except KeyError:
                            pass
                        try:
                            reference_frame = self._ros_parameters['robot_types'][
                                robot_data[cfname]['type']]['reference_frame']
                        except KeyError:
                            pass
                        try:
                            reference_frame = self._ros_parameters['robots'][
                                cfname]['reference_frame']
                        except KeyError:
                            pass
                    reference_frames.append(reference_frame)

        # initialize backend by dynamically loading the module
        backend_name = self._ros_parameters['sim']['backend']
        backend_dt = self._ros_parameters['sim']['backend_dt']
        self.backend_time_for_stamp = self._ros_parameters['sim']['backend_time_for_stamp']
        module = importlib.import_module('.backend.' + backend_name, package='crazyflie_sim')
        class_ = getattr(module, 'Backend')
        self.backend = class_(self, names, initial_states, dt=backend_dt)
        self.get_logger().info(f"Using backend: {backend_name}")
        self.get_logger().info(f"Number of backend robots: {len(self.backend.uavs)}")

        # initialize visualizations by dynamically loading the modules
        self.visualizations = []
        for vis_key in self._ros_parameters['sim']['visualizations']:
            if self._ros_parameters['sim']['visualizations'][vis_key]['enabled']:
                module = importlib.import_module(
                    '.visualization.' + str(vis_key), package='crazyflie_sim'
                )
                class_ = getattr(module, 'Visualization')
                if vis_key == 'rviz':
                    # special case for rviz, which needs the reference frames
                    vis = class_(
                        self,
                        self._ros_parameters['sim']['visualizations'][vis_key],
                        names,
                        initial_states,
                        reference_frames
                    )
                else:
                    vis = class_(
                        self,
                        self._ros_parameters['sim']['visualizations'][vis_key],
                        names,
                        initial_states
                    )
                self.visualizations.append(vis)

        controller_name = backend_name = self._ros_parameters['sim']['controller']

        # create robot SIL objects
        for name, initial_state in zip(names, initial_states):
            self.cfs[name] = CrazyflieSIL(
                name,
                initial_state.pos,
                controller_name,
                self.backend.time)
            
        self.pose_publishers = dict()
        self.odom_publishers = dict()
        for name, _ in self.cfs.items():
            self.get_logger().info(f"Name: {name}")
            pub = self.create_publisher(
                    String,
                    name + '/robot_description',
                    rclpy.qos.QoSProfile(
                        depth=1,
                        durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL))

            msg = String()
            msg.data = self._ros_parameters['robot_description'].replace('$NAME', name)
            pub.publish(msg)

            self.create_service(
                Empty,
                name + '/emergency',
                partial(self._emergency_callback, name=name)
            )
            self.create_service(
                Arm, name +
                "/arm", partial(self._arm_callback, name=name)
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
        self.create_service(Arm, 'all/arm', self._arm_callback)
        self.create_service(StartTrajectory,
                            'all/start_trajectory',
                            self._start_trajectory_callback)

        # This is the last service to announce.
        # Can be used to check if the server is fully available.
        self.create_service(Empty, 'all/emergency', self._emergency_callback)

        # step as fast as possible
        max_dt = 0.0 if 'max_dt' not in self._ros_parameters['sim'] \
            else self._ros_parameters['sim']['max_dt']
        self.timer = self.create_timer(max_dt, self._timer_callback)
        self.is_shutdown = False

    def on_shutdown_callback(self):
        if not self.is_shutdown:
            self.backend.shutdown()
            for visualization in self.visualizations:
                visualization.shutdown()

            self.is_shutdown = True

    def _timer_callback(self):
        # update setpoint
        states_desired = [cf.getSetpoint() for _, cf in self.cfs.items()]
        # execute the control loop
        actions = [cf.executeController() for _, cf in self.cfs.items()]
        disturbances = [cf.getDisturbance() for _, cf in self.cfs.items()]
        # execute the physics simulator
        self.get_logger().info(f"Actions desired: {actions}")
        states_next = self.backend.step(states_desired, actions, disturbances)

        # update the resulting state
        for state, (name, cf) in zip(states_next, self.cfs.items()):
            cf.setState(state)
            self._log_pose_data_callback(name, state)
            self._log_odom_data_callback(name, state)

        for vis in self.visualizations:
            vis.step(self.backend.time(), states_next, states_desired, actions)        

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
        if self.backend_time_for_stamp:
            time_elapsed_sim = self.backend.time()
            seconds = int(time_elapsed_sim)
            nanonseconds = int((time_elapsed_sim - seconds) * 1e9)
            msg.header.stamp = Time(seconds=seconds, nanoseconds=nanonseconds).to_msg()
        else:
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
    
    def _arm_callback(self, request, response, name='all'):
        """Service callback to arm the crazyflie."""
        self.get_logger().info(f'[{name}] arm: {request.arm}')

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


def main(args=None):

    rclpy.init(args=args)
    crazyflie_server = CrazyflieServer()
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
