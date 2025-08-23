import numpy as np

import robosuite.utils.transform_utils as T
from robosuite.controllers.base_controller import Controller
from robosuite.utils.buffers import DeltaBuffer, RingBuffer
from robosuite.utils.control_utils import *


class ForceController(Controller):
    def __init__(
        self,
        sim,
        eef_name,
        joint_indexes,
        actuator_range,
        input_max=1,
        input_min=-1,
        output_max=(5, 5, 5, 0.5, 0.5, 0.5),
        output_min=(-5, -5, -5, -0.5, -0.5, -0.5),
        kp=1,
        kdv=1,
        kda=1,
        damping_ratio=1,
        buffer_len=10,
        policy_freq=20,
        position_limits=None,
        orientation_limits=None,
        **kwargs,  # does nothing; used so no error raised when dict is passed with extra terms used previously
    ):
        self.wrench_buffer = RingBuffer(dim=6, length=buffer_len)
        self.ee_vel_buffer = DeltaBuffer(dim=6)
        self.err_buffer = DeltaBuffer(dim=6)

        super().__init__(
            sim,
            eef_name,
            joint_indexes,
            actuator_range,
        )

        # Control dimension
        self.control_dim = 6
        self.name_suffix = "FORCE"

        # input and output max and min (allow for either explicit lists or single numbers)
        self.input_max = self.nums2array(input_max, self.control_dim)
        self.input_min = self.nums2array(input_min, self.control_dim)
        self.output_max = self.nums2array(output_max, self.control_dim)
        self.output_min = self.nums2array(output_min, self.control_dim)

        # kp kd
        self.kp = self.nums2array(kp, 6)
        # self.kp[3:] = self.kp[3:] * 0.1
        self.kd = 2 * np.sqrt(self.kp) * damping_ratio
        self.kdv = kdv
        self.kda = kda
        # limits
        self.position_limits = np.array(position_limits) if position_limits is not None else position_limits
        self.orientation_limits = np.array(orientation_limits) if orientation_limits is not None else orientation_limits

        # control frequency
        self.control_freq = policy_freq

        self.goal_wrench = np.zeros(6)

    def update(self, force=False):
        super().update(force=force)
        # TODO: generalize
        site_id = self.sim.model.sensor_objid[0]
        site_xmat = self.sim.data.site_xmat[site_id].reshape(3, 3)
        wrench = np.copy(self.sim.data.sensordata)
        # wrench = -wrench
        wrench[:3] = site_xmat @ wrench[:3]
        wrench[3:] = site_xmat @ wrench[3:]
        self.wrench_buffer.push(wrench)

    def set_goal(self, action):
        # Update state
        self.update()

        self.goal_wrench = self.scale_action(action)

    def run_controller(self):
        """
        Returns:
             np.array: Command torques
        """
        # Update state
        self.update()
        actual_wrench = self.wrench_buffer.average
        wrench_error = self.goal_wrench - actual_wrench
        ee_vel = np.concatenate([self.ee_pos_vel, self.ee_ori_vel])
        vel_damping_force = -self.kdv * ee_vel
        acc_damping_force = -self.kda * self.ee_vel_buffer.delta
        vel_damping_force = np.clip(vel_damping_force, self.output_min, self.output_max)
        acc_damping_force = np.clip(acc_damping_force, self.output_min, self.output_max)
        desired_wrench = (
            self.kp * self.goal_wrench - self.kd * self.err_buffer.delta + vel_damping_force + acc_damping_force
        )

        self.err_buffer.push(wrench_error)
        self.ee_vel_buffer.push(ee_vel)
        # Gamma (without null torques) = J^T * F + gravity compensations
        self.torques = np.dot(self.J_full.T, desired_wrench) + self.torque_compensation

        # self.torques += nullspace_torques(
        #     self.mass_matrix, nullspace_matrix, self.initial_joint, self.joint_pos, self.joint_vel
        # )

        # Always run superclass call for any cleanups at the end
        super().run_controller()

        return self.torques

    def update_initial_joints(self, initial_joints):
        # First, update from the superclass method
        super().update_initial_joints(initial_joints)

        # We also need to reset the goal in case the old goals were set to the initial confguration
        self.reset_goal()

    def reset_goal(self):
        self.goal_wrench = np.zeros(6)

    @property
    def control_limits(self):
        low, high = self.input_min, self.input_max
        return low, high

    @property
    def name(self):
        return self.name_suffix
