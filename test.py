import numpy as np
import math

# EKF state covariance
Cx = np.diag([0.5, 0.5, np.deg2rad(30.0)])**2 # Change in covariance

#  Simulation parameter
Qsim = np.diag([0.2, np.deg2rad(1.0)])**2  # Sensor Noise
Rsim = np.diag([1.0, np.deg2rad(10.0)])**2 # Process Noise

DT = 0.1  # time tick [s]
SIM_TIME = 50.0  # simulation time [s]
MAX_RANGE = 20.0  # maximum observation range
M_DIST_TH = 2.0  # Threshold of Mahalanobis distance for data association.
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM state size [x,y]

def update(xEst, PEst, z, initP):
    """
    Performs the update step of EKF SLAM

    :param xEst:  nx1 the predicted pose of the system and the pose of the landmarks
    :param PEst:  nxn the predicted covariance
    :param z:     the measurements read at new position
    :param initP: 2x2 an identity matrix acting as the initial covariance
    :returns:     the updated state and covariance for the system
    """
    for iz in range(len(z[:, 0])):  # for each observation
        minid = search_correspond_LM_ID(xEst, PEst, z[iz, 0:2]) # associate to a known landmark

        nLM = calc_n_LM(xEst) # number of landmarks we currently know about

        if minid == nLM: # Landmark is a NEW landmark
            print("New LM")
            # Extend state and covariance matrix
            xAug = np.vstack((xEst, calc_LM_Pos(xEst, z[iz, :])))
            PAug = np.vstack((np.hstack((PEst, np.zeros((len(xEst), LM_SIZE)))),
                              np.hstack((np.zeros((LM_SIZE, len(xEst))), initP))))
            xEst = xAug
            PEst = PAug

        lm = get_LM_Pos_from_state(xEst, minid)
        y, S, H = calc_innovation(lm, xEst, PEst, z[iz, 0:2], minid)

        K = (PEst @ H.T) @ np.linalg.inv(S) # Calculate Kalman Gain
        xEst = xEst + (K @ y)
        PEst = (np.eye(len(xEst)) - (K @ H)) @ PEst

    xEst[2] = pi_2_pi(xEst[2])
    return xEst, PEst

def calc_innovation(lm, xEst, PEst, z, LMid):
    """
    Calculates the innovation based on expected position and landmark position

    :param lm:   landmark position
    :param xEst: estimated position/state
    :param PEst: estimated covariance
    :param z:    read measurements
    :param LMid: landmark id
    :returns:    returns the innovation y, and the jacobian H, and S, used to calculate the Kalman Gain
    """
    delta = lm - xEst[0:2]
    q = (delta.T @ delta)[0, 0]
    zangle = math.atan2(delta[1, 0], delta[0, 0]) - xEst[2, 0]
    zp = np.array([[math.sqrt(q), pi_2_pi(zangle)]])
    # zp is the expected measurement based on xEst and the expected landmark position

    y = (z - zp).T # y = innovation
    y[1] = pi_2_pi(y[1])

    H = jacobH(q, delta, xEst, LMid + 1)
    S = H @ PEst @ H.T + Cx[0:2, 0:2]

    return y, S, H

def jacobH(q, delta, x, i):
    """
    Calculates the jacobian of the measurement function

    :param q:     the range from the system pose to the landmark
    :param delta: the difference between a landmark position and the estimated system position
    :param x:     the state, including the estimated system position
    :param i:     landmark id + 1
    :returns:     the jacobian H
    """
    sq = math.sqrt(q)
    G = np.array([[-sq * delta[0, 0], - sq * delta[1, 0], 0, sq * delta[0, 0], sq * delta[1, 0]],
                  [delta[1, 0], - delta[0, 0], - q, - delta[1, 0], delta[0, 0]]])

    G = G / q
    nLM = calc_n_LM(x)
    F1 = np.hstack((np.eye(3), np.zeros((3, 2 * nLM))))
    F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * (i - 1))),
                    np.eye(2), np.zeros((2, 2 * nLM - 2 * i))))

    F = np.vstack((F1, F2))

    H = G @ F

    return H


def calc_n_LM(x):
    """
    Calculates the number of landmarks currently tracked in the state
    :param x: the state
    :returns: the number of landmarks n
    """
    n = int((len(x) - STATE_SIZE) / LM_SIZE)
    return n


def jacob_motion(x, u):
    """
    Calculates the jacobian of motion model.

    :param x: The state, including the estimated position of the system
    :param u: The control function
    :returns: G:  Jacobian
              Fx: STATE_SIZE x (STATE_SIZE + 2 * num_landmarks) matrix where the left side is an identity matrix
    """

    # [eye(3) [0 x y; 0 x y; 0 x y]]
    Fx = np.hstack((np.eye(STATE_SIZE), np.zeros(
        (STATE_SIZE, LM_SIZE * calc_n_LM(x)))))

    jF = np.array([[0.0, 0.0, -DT * u[0] * math.sin(x[2, 0])],
                   [0.0, 0.0, DT * u[0] * math.cos(x[2, 0])],
                   [0.0, 0.0, 0.0]],dtype=object)

    G = np.eye(STATE_SIZE) + Fx.T @ jF @ Fx
    if calc_n_LM(x) > 0:
        print(Fx.shape)
    return G, Fx,

def calc_LM_Pos(x, z):
    """
    Calculates the pose in the world coordinate frame of a landmark at the given measurement.

    :param x: [x; y; theta]
    :param z: [range; bearing]
    :returns: [x; y] for given measurement
    """
    zp = np.zeros((2, 1))

    zp[0, 0] = x[0, 0] + z[0] * math.cos(x[2, 0] + z[1])
    zp[1, 0] = x[1, 0] + z[0] * math.sin(x[2, 0] + z[1])
    #zp[0, 0] = x[0, 0] + z[0, 0] * math.cos(x[2, 0] + z[0, 1])
    #zp[1, 0] = x[1, 0] + z[0, 0] * math.sin(x[2, 0] + z[0, 1])

    return zp


def get_LM_Pos_from_state(x, ind):
    """
    Returns the position of a given landmark

    :param x:   The state containing all landmark positions
    :param ind: landmark id
    :returns:   The position of the landmark
    """
    lm = x[STATE_SIZE + LM_SIZE * ind: STATE_SIZE + LM_SIZE * (ind + 1), :]

    return lm


def search_correspond_LM_ID(xAug, PAug, zi):
    """
    Landmark association with Mahalanobis distance.

    If this landmark is at least M_DIST_TH units away from all known landmarks,
    it is a NEW landmark.

    :param xAug: The estimated state
    :param PAug: The estimated covariance
    :param zi:   the read measurements of specific landmark
    :returns:    landmark id
    """

    nLM = calc_n_LM(xAug)

    mdist = []

    for i in range(nLM):
        lm = get_LM_Pos_from_state(xAug, i)
        y, S, H = calc_innovation(lm, xAug, PAug, zi, i)
        mdist.append(y.T @ np.linalg.inv(S) @ y)

    mdist.append(M_DIST_TH)  # new landmark

    minid = mdist.index(min(mdist))

    return minid

def calc_input():
    v = 1.0  # [m/s]
    yawrate = 0.1  # [rad/s]
    u = np.array([[v, yawrate]]).T
    return u

def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi
