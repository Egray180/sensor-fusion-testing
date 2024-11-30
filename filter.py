from math import sin, cos, tan, sqrt, pi, asin, acos, atan2
import numpy as np

class KalmanFilter:

    a = 6378137.0 # semimajor axis length of WGS-84 ellipsoid
    e = 0.0818 # eccentricity of WGS-84 ellipsoid
    w_ie = 7.2921e-5 # earth angular velocity 

    def __init__(self, p0, v0, q0):
        self.pos = np.matrix(p0).T
        self.vel = np.matrix(v0).T
        self.quat = np.matrix(q0).T

        e_a, e_g, e_gps_p, e_gps_alt, e_gps_v = 5e-5, 5e-5, 1e-6, 1, 25e-4 

        self.P = np.matrix([[1e-6, 0, 0, 0, 0, 0, 0, 0, 0], [0, 1e-6, 0, 0, 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0, 0, 0, 0], \
                            [0, 0, 0, 25e-4, 0, 0, 0, 0, 0], [0, 0, 0, 0, 25e-4, 0, 0, 0, 0], [0, 0, 0, 0, 0, 25e-4, 0, 0, 0], \
                            [0, 0, 0, 0, 0, 0, 8e-3, 0, 0], [0, 0, 0, 0, 0, 0, 0, 8e-3, 0], [0, 0, 0, 0, 0, 0, 0, 0, 8e-3]]) # tilt errors in radians
        self.Q = np.matrix([[0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0], \
                            [0, 0, 0, e_a, 0, 0, 0, 0, 0], [0, 0, 0, 0, e_a, 0, 0, 0, 0], [0, 0, 0, 0, 0, e_a, 0, 0, 0], \
                            [0, 0, 0, 0, 0, 0, e_g, 0, 0], [0, 0, 0, 0, 0, 0, 0, e_g, 0], [0, 0, 0, 0, 0, 0, 0, 0, e_g]]) 
        self.R = np.matrix([[e_gps_p, 0, 0, 0, 0, 0], [0, e_gps_p, 0, 0, 0, 0], [0, 0, e_gps_alt, 0, 0, 0], \
                            [0, 0, 0, e_gps_v, 0, 0], [0, 0, 0, 0, e_gps_v, 0], [0, 0, 0, 0, 0, e_gps_v]])
        self.H = np.matrix([[180 / pi, 0, 0, 0, 0, 0, 0, 0, 0], [0, 180 / pi, 0, 0, 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0, 0, 0, 0], \
                            [0, 0, 0, 1, 0, 0, 0, 0, 0], [0, 0, 0, 0, 1, 0, 0, 0, 0], [0, 0, 0, 0, 0, 1, 0, 0, 0]])
        self.BNO_tilt_error = [8e-3, 8e-3, 8e-3]

    def predict(self, f_p, gyro_ip, T_s):
        '''
        Args:
            f_p: list of accelerometer readings 
            gyro_ip: list of gyroscope readings 
            T_s: time elapsed since last readings 
        '''
        # get rotation matrices from attitude quaternion 
        b1 = self.quat.item(0)
        b2 = self.quat.item(1)
        b3 = self.quat.item(2)
        b4 = self.quat.item(3)

        p1 = b1 ** 2
        p2 = b2 ** 2
        p3 = b3 ** 2
        p4 = b4 ** 2
        p5 = p2 + p3 

        if (p5 + p4 + p1 != 0): 
            p6 = 2 / (p5 + p4 + p1) 
        else: 
            p6 = 0

        R_11 = 1 - p6 * p5
        R_22 = 1 - p6 * (p1 + p3) 
        R_33 = 1 - p6 * (p1 + p2)
        p1 = p6 * b1
        p2 = p6 * b2 
        p5 = p6 * b3 * b4
        p6 = p1 * b2
        R_12 = p6 - p5
        R_21 = p6 + p5
        p5 = p2 * b4
        p6 = p1 * b3
        R_13 = p6 + p5 
        R_31 = p6 - p5
        p5 = p1 * b4
        p6 = p2 * b3
        R_23 = p6 - p5
        R_32 = p6 + p5

        R_p2n = np.matrix([[R_11, R_12, R_13], [R_21, R_22, R_23], [R_31, R_32, R_33]])

        R_n2p = R_p2n.I

        # fill state transition jacobian 
        lat = self.pos.item(0)
        h = self.pos.item(2)
        R_meridian = self.a * (1 - self.e ** 2) / (1 - self.e ** 2 * (sin(lat)) ** 2) ** 1.5 + h 
        R_transverse = self.a / (1 - self.e ** 2 * (sin(lat) ** 2)) ** 0.5 + h 
        R_e = sqrt(R_meridian * R_transverse) # assume that R_meridian ~ R_transverse ~ R_e 

        f_n = R_p2n * np.matrix(f_p).T 

        v_N = self.vel.item(0)
        v_E = self.vel.item(1)
        v_D = self.vel.item(2)
        f_N = f_n.item(0)
        f_E = f_n.item(1)
        f_D = f_n.item(2)

        omega_N = self.w_ie * cos(lat)
        omega_D = -self.w_ie * sin(lat)
        rho_N = v_E / R_e
        rho_E = -v_N / R_e
        rho_D = -v_E * tan(lat) / R_e
        w_N = omega_N + rho_N
        w_E = rho_E
        w_D = omega_D + rho_D

        k_D = v_D / R_e
        F_41 = -2 * omega_N * v_E - rho_N * v_E / cos(lat) ** 2 
        F_43 = rho_E * k_D - rho_N * rho_D
        F_51 = 2 * (omega_N * v_N + omega_D * v_D) + rho_N * v_N / cos(lat) ** 2 
        F_53 = -rho_E * rho_D * k_D * rho_N
        F_55 = k_D - rho_E * tan(lat)
        F_63 = rho_N ** 2 + rho_E ** 2 # - 2 * g / R_e

        F = np.matrix([[0, 0, rho_E / R_e, 1 / R_e, 0, 0, 0, 0, 0], [-rho_D / cos(lat), 0, -rho_N / (R_e * cos(lat)), 0, 1 / (R_e * cos(lat)), 0, 0, 0, 0], \
                       [0, 0, 0, 0, 0, -1, 0, 0, 0], [F_41, 0, F_43, k_D, 2 * w_D, -rho_E, 0, f_D, -f_E], [F_51, 0, F_53, -(w_D + omega_D), F_55, w_N + omega_N, -f_D, 0, f_N], \
                       [-2 * v_E * omega_D, 0, F_63, 2 * rho_E, -2 * w_N, 0, f_E, f_N, 0], [-omega_D, 0, rho_N / R_e, 0, -1 / R_e, 0, 0, w_D, -w_E], \
                       [0, 0, rho_E / R_e, 1 / R_e, 0, 0, -w_D, 0, w_N], [omega_N + rho_N / cos(lat) ** 2, 0, rho_D / R_e, 0, tan(lat) / R_e, 0, w_E, -w_N, 0]])

        # update states 
        p_dot = np.matrix([[1 / R_meridian, 0, 0], [0, 1 / (R_transverse * cos(lat)), 0], [0, 0, -1]]) * self.vel
        lat_dot = p_dot.item(0)
        long_dot = p_dot.item(1)
        g = np.matrix([0, 0, 9.8]).T # add centripetal term to this 
        v_dot = f_n + g + np.matrix([[0, -(long_dot + 2 * self.w_ie) * sin(lat), lat_dot], [(long_dot + 2 * self.w_ie) * sin(lat), 0, (long_dot + 2 * self.w_ie) * cos(lat)], \
                                     [-lat_dot, -(long_dot + 2 * self.w_ie) * cos(lat), 0]]) * self.vel
        
        gyro = np.matrix(gyro_ip).T - R_n2p * np.matrix([(long_dot + self.w_ie) * cos(lat), -lat_dot, -(long_dot + self.w_ie) * sin(lat)]).T
        p = gyro.item(0)
        q = gyro.item(1)
        r = gyro.item(2)
        norm = sqrt(p ** 2 + q ** 2 + r ** 2)
        omega = np.matrix([[0, r, -q, p], [-r, 0, p, q], [q, -p, 0, r], [-p, -q, -r, 0]]) # where quaternion is [vector part, scalar part]

        self.pos = self.pos + T_s * p_dot # add second order term 
        self.vel = self.vel + T_s * v_dot
        self.quat = (cos(0.5 * T_s * norm) * np.identity(4) + 1 / norm * sin(0.5 * T_s * norm) * omega) * self.quat

        self.P = (np.identity(9) + T_s * F) * self.P * (np.identity(9) + T_s * F).T + T_s * self.Q

    def update(self, gps, att): 
        '''
        Args:
            gps: list of position and velocity from gps 
            att: list of quaternions from BNO [vector part, scalar part]
        '''
        y = np.matrix(gps).T - self.H * np.matrix(self.pos.tolist() + self.vel.tolist() + [[0], [0], [0]])
        K = self.P * self.H.T * (self.H * self.P * self.H.T + self.R).I
        mod_list = (K * y).tolist()
        self.pos = self.pos + np.matrix(mod_list[:3])
        self.vel = self.vel + np.matrix(mod_list[3:6]) 
        self.P = (np.identity(9) - K * self.H) * self.P

        # attitude correction 
        q = self.getq()
        q1, q2, q3, q4 = q[0], q[1], q[2], q[3]
        Q_conj = np.matrix([[q4, q3, -q2, q1], [-q3, q4, q1, q2], [q2, -q1, q4, q3], [-q1, -q2, -q3, q4]])
        q_mod = vector_to_list(Q_conj.I * np.matrix(att).T)
        mag = 2 * acos(q_mod[3])
        q_mod_prime = vector_to_list(Q_conj.I * -np.matrix(att).T)
        mag_prime = 2 * acos(q_mod_prime[3])
        if mag > mag_prime:
            q_mod = q_mod_prime
            mag = mag_prime
        if mag < 1e-3:
            return
        tilt_error = list(map(lambda a: mag * a / sin(mag / 2), q_mod[:3]))
        # scale each angle in tilt_error
        temp_P = self.P.tolist()
        P_tilt_error = [temp_P[6][6], temp_P[7][7], temp_P[8][8]]
        tilt_error = [tilt_error[0] * P_tilt_error[0] / (P_tilt_error[0] + self.BNO_tilt_error[0]), \
                      tilt_error[1] * P_tilt_error[1] / (P_tilt_error[1] + self.BNO_tilt_error[1]), \
                      tilt_error[2] * P_tilt_error[2] / (P_tilt_error[2] + self.BNO_tilt_error[2])]
        # update attitude
        mag = sqrt(tilt_error[0] ** 2 + tilt_error[1] ** 2 + tilt_error[2] ** 2)
        q_mod = np.matrix(list(map(lambda a: sin(mag / 2) * a / mag, tilt_error)) + [cos(mag / 2)]).T
        self.quat = Q_conj * q_mod

        new_tilt_error = []
        for i in range(3):
            if P_tilt_error[i] <= self.BNO_tilt_error[i]:
                new_tilt_error.append(P_tilt_error[i])
            else:
                new_tilt_error.append(self.BNO_tilt_error[i])
        # update P (zones A & C)
        for i in range(6):
            for j in range(6, 9):
                temp_P[i][j] *= sqrt(new_tilt_error[j - 6] / P_tilt_error[j - 6])
                temp_P[j][i] = temp_P[i][j]
        # update P (zone B)
        for i in range (6, 9):
            for j in range(6, 9):
                temp_P[i][j] *= sqrt(new_tilt_error[i - 6] / P_tilt_error[i - 6]) * sqrt(new_tilt_error[j - 6] / P_tilt_error[j - 6])
        self.P = np.matrix(temp_P)
    
    def getp(self):
        return list(map(lambda a: a[0], self.pos.tolist()))

    def getv(self):
        return list(map(lambda a: a[0], self.vel.tolist()))
    
    def getq(self):
        return list(map(lambda a: a[0], self.quat.tolist()))

def vector_to_list(v):
    return list(map(lambda a: a[0], v.tolist()))
    
def quat_to_ypr(q):
    yaw   = atan2(2.0 * (q[0] * q[1] + q[3] * q[2]), q[3] * q[3] + q[0] * q[0] - q[1] * q[1] - q[2] * q[2])
    pitch = -asin(2.0 * (q[0] * q[2] - q[3] * q[1]))
    roll  = atan2(2.0 * (q[3] * q[0] + q[1] * q[2]), q[3] * q[3] - q[0] * q[0] - q[1] * q[1] + q[2] * q[2])
    return [yaw, pitch, roll]

