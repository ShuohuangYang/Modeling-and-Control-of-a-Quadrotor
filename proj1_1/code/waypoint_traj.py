import numpy as np


class WaypointTraj(object):
    """

    """

    def __init__(self, points):
        """
        This is the constructor for the Trajectory object. A fresh trajectory
        object will be constructed before each mission. For a waypoint
        trajectory, the input argument is an array of 3D destination
        coordinates. You are free to choose the times of arrival and the path
        taken between the points in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Inputs:
            points, (N, 3) array of N waypoint coordinates in 3D
        """

        # STUDENT CODE HERE
        self.row, self.col = points.shape
        self.points = points

    def update(self, t):
        """
        Given the present time, return the desired flat output and derivatives.

        Inputs
            t, time, s
        Outputs
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s
        """
        x = np.zeros((3,))
        x_dot = np.zeros((3,))
        x_ddot = np.zeros((3,))
        x_dddot = np.zeros((3,))
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0

        # STUDENT CODE HERE
        num_pt = self.row

        vel = 1.5
        x_int_pt = self.points[0:num_pt - 1, 0]
        x_fin_pt = self.points[1:num_pt, 0]

        y_int_pt = self.points[0:num_pt - 1, 1]
        y_fin_pt = self.points[1:num_pt, 1]

        z_int_pt = self.points[0:num_pt - 1, 2]
        z_fin_pt = self.points[1:num_pt, 2]

        t_spend = []
        t_spend.append(0)
        # estimate time spent on each path
        for i in range(num_pt - 1):
            x_dist = np.abs(x_fin_pt[i] - x_int_pt[i])
            y_dist = np.abs(y_fin_pt[i] - y_int_pt[i])
            z_dist = np.abs(z_fin_pt[i] - z_int_pt[i])

            t_x = x_dist / vel
            t_y = y_dist / vel
            t_z = z_dist / vel

            t_max = np.max([t_x, t_y, t_z])
            t_spend.append(t_max + t_spend[-1])

        t_0 = t_spend[0:len(t_spend) - 1]
        t_f = t_spend[1:len(t_spend)]

        # determine which path the robot is currently at
        num_path = 0  # time interval
        for i in range(num_pt):
            if t > t_spend[i]:
                num_path = i

        if num_path >= num_pt - 1:  # the last point or exceed
            x = self.points[-1, :]
        else:
            t0 = t_0[num_path]
            tf = t_f[num_path]
            xi = x_int_pt[num_path]
            xf = x_fin_pt[num_path]
            yi = y_int_pt[num_path]
            yf = y_fin_pt[num_path]
            zi = z_int_pt[num_path]
            zf = z_fin_pt[num_path]

            A = np.array([[1, t0, t0 ** 2, t0 ** 3],
                          [0, 1, 2 * t0, 3 * t0 ** 2],
                          [1, tf, tf ** 2, tf ** 3],
                          [0, 1, 2 * tf, 3 * tf ** 2]])
            B_x = np.array([xi, 0, xf, 0])
            x_a = np.dot(np.linalg.inv(A), np.transpose(B_x))

            B_y = np.array([yi, 0, yf, 0])
            y_a = np.dot(np.linalg.inv(A), np.transpose(B_y))

            B_z = np.array([zi, 0, zf, 0])
            z_a = np.dot(np.linalg.inv(A), np.transpose(B_z))

            my_x = x_a[0] + x_a[1] * t + x_a[2] * t ** 2 + x_a[3] * t ** 3
            my_x_dot = x_a[1] + 2 * x_a[2] * t + 3 * x_a[3] * t ** 2
            my_x_ddot = 2 * x_a[2] + 6 * x_a[3] * t
            my_x_dddot = 6 * x_a[3]
            my_x_ddddot = 0

            y = y_a[0] + y_a[1] * t + y_a[2] * t ** 2 + y_a[3] * t ** 3
            y_dot = y_a[1] + 2 * y_a[2] * t + 3 * y_a[3] * t ** 2
            y_ddot = 2 * y_a[2] + 6 * y_a[3] * t
            y_dddot = 6 * y_a[3]
            y_ddddot = 0

            z = z_a[0] + z_a[1] * t + z_a[2] * t ** 2 + z_a[3] * t ** 3
            z_dot = z_a[1] + 2 * z_a[2] * t + 3 * z_a[3] * t ** 2
            z_ddot = 2 * z_a[2] + 6 * z_a[3] * t
            z_dddot = 6 * z_a[3]
            z_ddddot = 0

            x = np.array([my_x, y, z])
            x_dot = np.array([my_x_dot, y_dot, z_dot])
            x_ddot = np.array([my_x_ddot, y_ddot, z_ddot])
            x_dddot = np.array([my_x_dddot, y_dddot, z_dddot])
            x_ddddot = np.array([my_x_ddddot, y_ddddot, z_ddddot])

        flat_output = {'x': x, 'x_dot': x_dot, 'x_ddot': x_ddot, 'x_dddot': x_dddot, 'x_ddddot': x_ddddot,
                       'yaw': yaw, 'yaw_dot': yaw_dot}
        return flat_output
