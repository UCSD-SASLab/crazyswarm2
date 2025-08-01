from pathlib import Path

import numpy as np
from rclpy.node import Node
from rclpy.time import Time
import dynobench
from rosgraph_msgs.msg import Clock

# import sys
# sys.path.append("/home/whoenig/projects/dynobench/build")

from ..sim_data_types import Action, State


class Backend:
    """Backend that uses newton-euler rigid-body dynamics implemented in numpy."""

    def __init__(self, node: Node, names: list[str], states: list[State], dt: float):
        self.node = node
        self.names = names
        self.clock_publisher = node.create_publisher(Clock, 'clock', 10)
        self.t = 0
        self.dt = dt

        self.uavs = []
        for state in states:
            uav = Quadrotor(state)
            self.uavs.append(uav)

    def time(self) -> float:
        return self.t

    def step(self, states_desired: list[State], actions: list[Action], disturbances: list[State]) -> list[State]:
        # advance the time
        self.t += self.dt

        next_states = []

        for uav, action, disturbance in zip(self.uavs, actions, disturbances):
            uav.step(action, disturbance, self.dt)
            next_states.append(uav.state)

        # print(states_desired, actions, next_states)
        # publish the current clock
        clock_message = Clock()
        clock_message.clock = Time(seconds=self.time()).to_msg()
        self.clock_publisher.publish(clock_message)

        return next_states

    def shutdown(self):
        pass


# convert RPM -> Force
def rpm_to_force(rpm):
    # polyfit using data and scripts from https://github.com/IMRCLab/crazyflie-system-id
    p = [2.55077341e-08, -4.92422570e-05, -1.51910248e-01]
    force_in_grams = np.polyval(p, rpm)
    force_in_newton = force_in_grams * 9.81 / 1000.0
    return np.maximum(force_in_newton, 0)


def sim_state2dynobench_state(state):
    result = np.empty(13)
    result[0:3] = state.pos
    result[3:6] = state.quat[1:]
    result[6] = state.quat[0]
    result[7:10] = state.vel
    result[10:13] = state.omega
    return result


def dynobench_state2sim_state(state):
    result = State()
    result.pos = state[0:3]
    result.quat = np.concatenate((state[6:7], state[3:6]))
    result.vel = state[7:10]
    result.omega = state[10:13]
    return result


class Quadrotor:
    """Basic rigid body quadrotor model (no drag) using numpy and rowan."""

    def __init__(self, state):
        self.uav = dynobench.robot_factory(
            str((Path(__file__).parent / 'data/dynobench/crazyflie2.yaml').resolve()), [], [])
        self.state = state

    def step(self, action, disturbance, dt):

        # m: 0.034
        # max_f: 1.3
        force_in_newton = rpm_to_force(action.rpm)
        normalized_force = force_in_newton / (1.3 * (0.034 / 4) * 9.81)

        xnext = np.zeros(13)
        self.uav.step(xnext, sim_state2dynobench_state(self.state), normalized_force, dt)
        xnext[0:3] += disturbance.pos * dt
        xnext[7:10] += disturbance.vel * dt
        self.state = dynobench_state2sim_state(xnext)

        # if we fall below the ground, set velocities to 0
        if self.state.pos[2] < 0:
            self.state.pos[2] = 0
            self.state.vel = [0, 0, 0]
            self.state.omega = [0, 0, 0]
