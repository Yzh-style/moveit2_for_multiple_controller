import math
from numpy import mat, block, zeros, array
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R

# Function to return to the point 1700 and 650
def return_origin(a, b, alpha, q):
    print("return_origin")
    nq, success = next_point(1700, 650, a, b, alpha)
    if success:
        alpha = path_plan(nq, a, b, alpha)
    return alpha

# Quaternion-based rotation matrix for inverse kinematics
def quaternion_rotation_matrix(angle, axis):
    r = R.from_quat([axis[0] * math.sin(angle / 2),
                     axis[1] * math.sin(angle / 2),
                     axis[2] * math.sin(angle / 2),
                     math.cos(angle / 2)])
    return r.as_matrix()

# Function to find the angle after calculation
def find_angle(sin_theta, cos_theta):
    angle_rad = math.atan2(sin_theta, cos_theta)
    return angle_rad

# Calculation in X
def xE_func(q, xE, yE, alpha, a, b):
    R0_w = quaternion_rotation_matrix(alpha[0], [0, 0, 1])
    R1_0 = quaternion_rotation_matrix(alpha[1], [0, 0, 1]) @ quaternion_rotation_matrix(q[0], [0, 0, 1])
    R2_1 = quaternion_rotation_matrix(alpha[2], [0, 0, 1]) @ quaternion_rotation_matrix(q[1], [0, 0, 1])
    R3_2 = quaternion_rotation_matrix(alpha[3], [0, 0, 1]) @ quaternion_rotation_matrix(q[2], [0, 0, 1])
    R4_3 = quaternion_rotation_matrix(alpha[4], [0, 0, 1])

    P0_w = mat([[a[0]], [b[0]], [0]])
    P1_0 = mat([[a[1]], [b[1]], [0]])
    P2_1 = mat([[a[2]], [b[2]], [0]])
    P3_2 = mat([[a[3]], [b[3]], [0]])
    P4_3 = mat([[a[4]], [b[4]], [0]])

    T0_w = block([[R0_w, P0_w], [zeros((1, 3)), 1]])
    T1_0 = block([[R1_0, P1_0], [zeros((1, 3)), 1]])
    T2_1 = block([[R2_1, P2_1], [zeros((1, 3)), 1]])
    T3_2 = block([[R3_2, P3_2], [zeros((1, 3)), 1]])
    T4_3 = block([[R4_3, P4_3], [zeros((1, 3)), 1]])

    T1_w = T0_w @ T1_0
    T2_w = T1_w @ T2_1
    T3_w = T2_w @ T3_2
    T4_w = T3_w @ T4_3

    func_x = T4_w[0, 3] - xE
    return func_x

# Calculation in Y
def yE_func(q, xE, yE, alpha, a, b):
    R0_w = quaternion_rotation_matrix(alpha[0], [0, 0, 1])
    R1_0 = quaternion_rotation_matrix(alpha[1], [0, 0, 1]) @ quaternion_rotation_matrix(q[0], [0, 0, 1])
    R2_1 = quaternion_rotation_matrix(alpha[2], [0, 0, 1]) @ quaternion_rotation_matrix(q[1], [0, 0, 1])
    R3_2 = quaternion_rotation_matrix(alpha[3], [0, 0, 1]) @ quaternion_rotation_matrix(q[2], [0, 0, 1])
    R4_3 = quaternion_rotation_matrix(alpha[4], [0, 0, 1])

    P0_w = mat([[a[0]], [b[0]], [0]])
    P1_0 = mat([[a[1]], [b[1]], [0]])
    P2_1 = mat([[a[2]], [b[2]], [0]])
    P3_2 = mat([[a[3]], [b[3]], [0]])
    P4_3 = mat([[a[4]], [b[4]], [0]])

    T0_w = block([[R0_w, P0_w], [zeros((1, 3)), 1]])
    T1_0 = block([[R1_0, P1_0], [zeros((1, 3)), 1]])
    T2_1 = block([[R2_1, P2_1], [zeros((1, 3)), 1]])
    T3_2 = block([[R3_2, P3_2], [zeros((1, 3)), 1]])
    T4_3 = block([[R4_3, P4_3], [zeros((1, 3)), 1]])

    T1_w = T0_w @ T1_0
    T2_w = T1_w @ T2_1
    T3_w = T2_w @ T3_2
    T4_w = T3_w @ T4_3

    func_y = T4_w[1, 3] - yE
    return func_y

# Function to calculate the next point
def next_point(xE, yE, a, b, alpha):
    print("goal:", xE, yE)

    cons = ({
        'type': 'eq', 'fun': xE_func, 'args': (xE, yE, alpha, a, b)
    }, {
        'type': 'eq', 'fun': yE_func, 'args': (xE, yE, alpha, a, b)
    })

    fun = lambda q: q[0]**2 + q[1]**2 + q[2]**2
    q0 = array((0.0, 0.0, 0.0))
    bnds = ((-math.pi / 2, math.pi / 2), (-math.pi / 2, math.pi / 2), (-math.pi / 2, math.pi / 2))

    res = minimize(fun, q0, bounds=bnds, constraints=cons)

    print(res)
    success = res.success
    q = [res.x[0], res.x[1], res.x[2]]
    
    return q, success

# Function to calculate the slope for the next point
def slope_next_point(slope, xE, yE):
    slope = math.degrees(slope)
    xE += 200 * math.sin(slope)
    yE += -200 * math.cos(slope)
    return xE, yE

# Function to find the inside angles for the joints
def path_plan(q, a, b, alpha):
    R0_w = quaternion_rotation_matrix(alpha[0], [0, 0, 1])
    R1_0 = quaternion_rotation_matrix(alpha[1], [0, 0, 1]) @ quaternion_rotation_matrix(q[0], [0, 0, 1])
    R2_1 = quaternion_rotation_matrix(alpha[2], [0, 0, 1]) @ quaternion_rotation_matrix(q[1], [0, 0, 1])
    R3_2 = quaternion_rotation_matrix(alpha[3], [0, 0, 1]) @ quaternion_rotation_matrix(q[2], [0, 0, 1])
    R4_3 = quaternion_rotation_matrix(alpha[4], [0, 0, 1])

    alpha = [
        0,
        find_angle(R1_0[1, 0], R1_0[1, 1]),
        find_angle(R2_1[1, 0], R2_1[1, 1]),
        find_angle(R3_2[1, 0], R3_2[1, 1]),
        0
    ]
    return alpha
