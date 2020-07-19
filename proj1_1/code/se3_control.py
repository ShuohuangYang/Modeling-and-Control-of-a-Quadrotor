import numpy as np
from scipy.spatial.transform import Rotation


class SE3Control(object):
    """

    """

    def __init__(self, quad_params):
        """
        This is the constructor for the SE3Control object. You may instead
        initialize any parameters, control gain values, or private state here.

        For grading purposes the controller is always initialized with one input
        argument: the quadrotor's physical parameters. If you add any additional
        input arguments for testing purposes, you must provide good default
        values!

        Parameters:
            quad_params, dict with keys specified by crazyflie_params.py

        """

        # Quadrotor physical parameters.
        self.mass = quad_params['mass']  # kg
        self.Ixx = quad_params['Ixx']  # kg*m^2
        self.Iyy = quad_params['Iyy']  # kg*m^2
        self.Izz = quad_params['Izz']  # kg*m^2
        self.arm_length = quad_params['arm_length']  # meters
        self.rotor_speed_min = quad_params['rotor_speed_min']  # rad/s
        self.rotor_speed_max = quad_params['rotor_speed_max']  # rad/s
        self.k_thrust = quad_params['k_thrust']  # N/(rad/s)**2
        self.k_drag = quad_params['k_drag']  # Nm/(rad/s)**2
        self.L = self.arm_length
        # You may define any additional constants you like including control gains.
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz]))  # kg*m^2
        self.gamma = self.k_drag / self.k_thrust

        self.g = 9.81  # m/s^2

        # STUDENT CODE HERE

    def update(self, t, state, flat_output):
        """
        This function receives the current time, true state, and desired flat
        outputs. It returns the command inputs.

        Inputs:
            t, present time in seconds
            state, a dict describing the present state with keys
                x, position, m
                v, linear velocity, m/s
                q, quaternion [i,j,k,w]
                w, angular velocity, rad/s
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s

        Outputs:
            control_input, a dict describing the present computed control inputs with keys
                cmd_motor_speeds, rad/s
                cmd_thrust, N (for debugging and laboratory; not used by simulator)
                cmd_moment, N*m (for debugging; not used by simulator)
                cmd_q, quaternion [i,j,k,w] (for laboratory; not used by simulator)
        """
        cmd_motor_speeds = np.zeros((4,))
        cmd_thrust = 0
        cmd_moment = np.zeros((3,))
        cmd_q = np.zeros((4,))

        # STUDENT CODE HERE
        # gain

        # position
        # kp_x = 3.2
        # kp_y = 3.3
        # kp_z = 5
        kp_x = 3.3
        kp_y = 3.3
        kp_z = 5

        kd_x = 1.45 * np.sqrt(kp_x)
        kd_y = 1.45 * np.sqrt(kp_y)
        kd_z = 1.50 * np.sqrt(kp_z)
        # kd_x = 4.0
        # kd_y = 4
        # kd_z = 3.5

        # orientation
        # kR_x = 80
        # kR_y = 80
        # kR_z = 5
        kR_x = 90
        kR_y = 70
        kR_z = 10

        kw_x = 2 * np.sqrt(40)
        kw_y = 2 * np.sqrt(40)
        kw_z = 2 * np.sqrt(10)
        # kw_x = 16
        # kw_y = 16
        # kw_z = 7

        k_d = np.array([kd_x, kd_y, kd_z])
        k_p = np.array([kp_x, kp_y, kp_z])
        k_R = np.array([kR_x, kR_y, kR_z])
        k_w = np.array([kw_x, kw_y, kw_z])
        k_d = np.diag(k_d)
        k_p = np.diag(k_p)
        k_R = np.diag(k_R)
        k_w = np.diag(k_w)

        # coefficient
        # k_F = self.k_thrust
        # k_M = self.k_drag
        #
        # gamma = k_M / k_F

        # copy values from input
        x_state = state.get('x')
        v = state.get('v')
        q = state.get('q')
        w = state.get('w')
        x_traj = flat_output.get('x')
        x_dot = flat_output.get('x_dot')
        x_ddot = flat_output.get('x_ddot')
        x_dddot = flat_output.get('x_dddot')
        x_ddddot = flat_output.get('x_ddddot')
        yaw = flat_output.get('yaw')
        yaw_dot = flat_output.get('yaw_dot')

        # calculate control input u1
        r_ddot_des = x_ddot - np.matmul(k_d, (v - x_dot)) - np.matmul(k_p, (x_state - x_traj))
        F_des = self.mass * r_ddot_des + np.array([0, 0, self.mass * self.g])
        R_f = Rotation.from_quat(q)
        R = R_f.as_matrix()

        b3 = np.matmul(R, np.array([0, 0, 1]))
        u1 = np.matmul(b3, F_des)

        # calculate control input u2
        b3_des = F_des / np.linalg.norm(F_des)
        a_yaw = np.array([np.cos(yaw), np.sin(yaw), 0])
        b2_des = np.cross(b3_des, a_yaw) / np.linalg.norm(np.cross(b3_des, a_yaw))
        R_des = np.array([np.cross(b2_des, b3_des), b2_des, b3_des])
        R_des_transp = np.transpose(R_des)
        e_R = (np.matmul(np.transpose(R_des_transp), R) - np.matmul(np.transpose(R), R_des_transp)) / 2
        e_R = np.array(e_R[[2, 0, 1], [1, 2, 0]])

        # since w_des, the desired angular velocities canbe computed from the output of the trajectort generator  zT
        # and its derivatives, but settiting them to zero will work for the purpose of this project
        w_des = 0
        e_w = w - w_des

        ke_R = -np.matmul(k_R, e_R)
        print("ke_R")
        print(ke_R)
        ke_w = -np.matmul(k_w, e_w)
        print("ke_w")
        print(ke_w)
        error_mat = ke_R + ke_w  # this was typo as ke_R - ke_w, therefore have bug, now resolved
        u2 = np.matmul(self.inertia, error_mat)
        print("u2")
        print(u2)
        u1 = np.array([u1])
        u = np.concatenate((u1, u2), axis=0)
        mat = np.array([[1, 1, 1, 1], [0, self.L, 0, -self.L], [-self.L, 0, self.L, 0],
                        [self.gamma, -self.gamma, self.gamma, -self.gamma]])
        F_cal = np.matmul(np.linalg.inv(mat), u)
        F_sign = np.sign(F_cal)
        F_abs = np.abs(F_cal)
        # print(F_cal)
        # print(F_abs)
        # print(F_abs * F_sign)
        cmd_motor_speeds = np.sqrt(
            F_abs / self.k_thrust)  # only want positive speed due to reality and practical manner
        print(cmd_motor_speeds)
        cmd_thrust = u1
        cmd_moment = u2
        # cmd_q = R_des_transp  to quatenion

        control_input = {'cmd_motor_speeds': cmd_motor_speeds,
                         'cmd_thrust': cmd_thrust,
                         'cmd_moment': cmd_moment,
                         'cmd_q': cmd_q}
        return control_input
