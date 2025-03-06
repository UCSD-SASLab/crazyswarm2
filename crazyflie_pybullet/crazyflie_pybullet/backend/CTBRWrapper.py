from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel
import numpy as np


class CTBRWrapper(DSLPIDControl):
    """Simple wrapper of the DSLPIDControl class for controlling a Crazyflie drone."""

    def __init__(self, drone_model: DroneModel, g, max_thrust):
        super().__init__(drone_model=drone_model, g=g)
        self.max_thrust = max_thrust

    def thrust_rpy_ctrl(self, thrust, des_rpy, control_timestep, quat):
        thrust = max(0, min(thrust, self.max_thrust))
        cf_thrust = (np.sqrt(thrust / (4 * self.KF)) - self.PWM2RPM_CONST) / self.PWM2RPM_SCALE
        rpm = self._dslPIDAttitudeControl(control_timestep, cf_thrust, quat, des_rpy, np.zeros(3))

        return rpm