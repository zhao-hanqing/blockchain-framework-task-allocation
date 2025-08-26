import math
from math import radians


# EKF state covariance
    # EKF state covariance
Cx = [[0.5 ** 2, 0, 0],
      [0, 0.5 ** 2, 0],
      [0, 0, radians(30.0) ** 2]]  # Change in covariance

# Simulation parameter
Qsim = [[0, 0],
        [0, 0]]  # Sensor Noise

Rsim = [[0, 0],
        [0, 0]]  # Process Noise
DT = 0.1  # time tick [s]
SIM_TIME = 50.0  # simulation time [s]
MAX_RANGE = 20.0  # maximum observation range
M_DIST_TH = 2.0  # Threshold of Mahalanobis distance for data association.
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM state size [x,y]


def matrix_multiply(A, B):
    return [[sum(a * b for a, b in zip(A_row, B_col)) for B_col in zip(*B)] for A_row in A]

def scalar_multiply(scalar, matrix):
    return [[scalar * element for element in row] for row in matrix]

def transpose_matrix(matrix):
    return list(map(list, zip(*matrix)))

def get_matrix_minor(matrix, row, col):
    matrix = [list(x) for x in list(matrix)]
    minor = [row[:col] + row[col + 1:] for row in (list(matrix[:row]) + list(matrix[row + 1:]))]
    return minor

def get_matrix_det(matrix):
    if len(matrix) == 2:
        return matrix[0][0]*matrix[1][1] - matrix[0][1]*matrix[1][0]

    determinant = 0
    for c in range(len(matrix)):
        determinant += ((-1) ** c) * matrix[0][c] * get_matrix_det(get_matrix_minor(matrix, 0, c))
    return determinant

def get_matrix_inv(matrix):
    determinant = get_matrix_det(matrix)
    if len(matrix) == 2:
        return [[matrix[1][1]/determinant, -1*matrix[0][1]/determinant],
                [-1*matrix[1][0]/determinant, matrix[0][0]/determinant]]

    elif len(matrix) == 3:
        determinant = get_matrix_det(matrix)
        if determinant == 0:
            return None
        else:
            return [[((matrix[1][1] * matrix[2][2] - matrix[2][1] * matrix[1][2]) / determinant),
                     (-1 * (matrix[0][1] * matrix[2][2] - matrix[0][2] * matrix[2][1]) / determinant),
                     ((matrix[0][1] * matrix[1][2] - matrix[0][2] * matrix[1][1]) / determinant)],

                    [(-1 * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) / determinant),
                     ((matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[2][0]) / determinant),
                     (-1 * (matrix[0][0] * matrix[1][2] - matrix[0][2] * matrix[1][0]) / determinant)],

                    [((matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]) / determinant),
                     (-1 * (matrix[0][0] * matrix[2][1] - matrix[0][1] * matrix[2][0]) / determinant),
                     ((matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]) / determinant)]]

    cofactors = []
    for r in range(len(matrix)):
        cofactorRow = []
        for c in range(len(matrix)):
            minor = get_matrix_minor(matrix, r, c)
            cofactorRow.append(((-1) ** (r + c)) * get_matrix_det(minor))
        cofactors.append(cofactorRow)
    cofactors = transpose_matrix(cofactors)
    for r in range(len(cofactors)):
        for c in range(len(cofactors)):
            cofactors[r][c] = cofactors[r][c]/determinant
    return cofactors

def matrix_add(A, B):
    return [[A[i][j] + B[i][j]  for j in range(len(A[0]))] for i in range(len(A))]

def scalar_matrix_multiply(scalar, matrix):
    return [[scalar * element for element in row] for row in matrix]

def identity_matrix(size):
    return [[1 if row == column else 0 for column in range(size)] for row in range(size)]

def matrix_subtract(A, B):
    return [[A[i][j] - B[i][j]  for j in range(len(A[0]))] for i in range(len(A))]

def zeros_matrix(rows, cols):
    return [[0]*cols for _ in range(rows)]

def horizontal_stack(A, B):
    return [list(row1) + list(row2) for row1, row2 in zip(A, B)]
def squeeze_list(old_list):
    return [[element[0] for element in sublist] for sublist in old_list]

def vertical_stack(A, B):
    return list(A) + list(B)

def ekf_slam(xEst, PEst, u, z, pos_measure, pos_internal_id, slam_frozen_lm_id=[]):
    """
    Performs an iteration of EKF SLAM from the available information.

    :param xEst: the belief in last position
    :param PEst: the uncertainty in last position
    :param u:    the control function applied to the last position
    :param z:    measurements at this step
    :returns:    the next estimated position and associated covariance
    """
    S = STATE_SIZE

    # Predict
    xEst, PEst, G, Fx = predict(xEst, PEst, u)
    initP = identity_matrix(2)
    pred_pos = xEst[:2]

    # Update
    # zr[2] is local index
    xEst, PEst, pos_internal_id = update(xEst, PEst, u, list(z), initP, list(pos_measure),[int(zr[2]) for zr in z], pos_internal_id, slam_frozen_lm_id)

    return xEst, PEst, pos_internal_id, pred_pos

def predict(xEst, PEst, u):
    """
    Performs the prediction step of EKF SLAM

    :param xEst: nx1 state vector
    :param PEst: nxn covariance matrix
    :param u:    2x1 control vector
    :returns:    predicted state vector, predicted covariance, jacobian of control vector, transition fx
    """
    S = STATE_SIZE
    G, Fx = jacob_motion(xEst[0:S], u)
    xEst[0:S] = motion_model(xEst[0:S], u)

    G_T = transpose_matrix(G)
    Fx_T = transpose_matrix(Fx)

    PEst_temp = [row[0:S] for row in PEst[0:S]]
    PEst_update = matrix_add(matrix_multiply(matrix_multiply(G_T, PEst_temp), G), matrix_multiply(matrix_multiply(Fx_T, Cx), Fx))
    for i in range(S):
        for j in range(S):
            PEst[i][j] = PEst_update[i][j]
    return xEst, PEst, G, Fx
def motion_model(x, u):
    """
    Computes the motion model based on current state and input function.

    :param x: 3x1 pose estimation
    :param u: 2x1 control input [v; w]
    :returns: the resulting state after the control function is applied
    """
    F = identity_matrix(3)

    B = [[DT * math.cos(x[2][0]), 0],
         [DT * math.sin(x[2][0]), 0],
         [0.0, DT]]

    x = matrix_add(matrix_multiply(F, x), matrix_multiply(B, u))
    return x

def update(xEst, PEst, u, z, initP, pos_obs=[], pos_ids = [], pos_internal_ids = [], frozen_lm_ids = []):
    """
    Performs the update step of EKF SLAM

    :param xEst:  nx1 the predicted pose of the system and the pose of the landmarks
    :param PEst:  nxn the predicted covariance
    :param u:     2x1 the control function
    :param z:     the measurements read at new position
    :param initP: 2x2 an identity matrix acting as the initial covariance
    :returns:     the updated state and covariance for the system
    """

    for iz in range(len(z)):  # for each observation
        this_lm_id = pos_ids[iz]

        nLM = calc_n_LM(xEst)  # number of landmarks we currently know about

        if this_lm_id not in pos_internal_ids:  # Landmark is a NEW landmark
            #print("New LM: ", this_lm_id, pos_internal_ids)
            # Extend state and covariance matrix
            xAug = vertical_stack(xEst, calc_LM_Pos(xEst, z[iz]))
            zeros1 = [[0 for _ in range(LM_SIZE)] for _ in range(len(xEst))]
            zeros2 = [[0 for _ in range(len(xEst))] for _ in range(LM_SIZE)]
            PAug = vertical_stack(horizontal_stack(PEst, zeros1), horizontal_stack(zeros2, initP))
            xEst = xAug
            PEst = PAug
            pos_internal_ids.append(this_lm_id) #add this id into internal id list

        lm = get_LM_Pos_from_state(xEst, pos_internal_ids.index(this_lm_id))
        if this_lm_id in frozen_lm_ids:
            isFrozen=True
        else:
            isFrozen=False
        y, S, H = calc_innovation(lm, xEst, PEst, z[iz][0:2], pos_internal_ids.index(this_lm_id), isFrozen)
        #print(" actual pose: ", pos_obs, "xest: ", xEst)
        K = matrix_multiply(matrix_multiply(PEst, transpose_matrix(H)), get_matrix_inv(S))
        xEst = matrix_add(xEst, matrix_multiply(K, y))
        PEst = matrix_multiply(matrix_subtract(identity_matrix(len(xEst)), matrix_multiply(K, H)), PEst)

    if pos_obs:
        y, S, H = calc_innovation_pos(xEst, PEst, list(pos_obs))
        K = matrix_multiply(matrix_multiply(PEst, transpose_matrix(H)), get_matrix_inv(S))
        xEst = matrix_add(xEst, matrix_multiply(K, y))
        PEst = matrix_multiply(matrix_subtract(identity_matrix(len(xEst)), matrix_multiply(K, H)), PEst)

    xEst[2][0] = pi_2_pi(xEst[2][0])
    return xEst, PEst, pos_internal_ids


def calc_innovation(lm, xEst, PEst, z, LMid, isFrozen):
    """
    Calculates the innovation based on expected position and landmark position

    :param lm:   landmark position
    :param xEst: estimated position/state
    :param PEst: estimated covariance
    :param z:    read measurements
    :param LMid: landmark id
    :returns:    returns the innovation y, and the jacobian H, and S, used to calculate the Kalman Gain
    """
    delta = matrix_subtract(lm, xEst[0:2])
    q = matrix_multiply(transpose_matrix(delta), delta)[0][0]
    # delta = lm - xEst[0:2]
    # q = (delta.T @ delta)[0, 0]
    zangle = math.atan2(delta[1][0], delta[0][0]) - xEst[2][0]
    zp = [[math.sqrt(q), pi_2_pi(zangle)]]

    # zp is the expected measurement based on xEst and the expected landmark position

    y = transpose_matrix(matrix_subtract([list(z)],zp)) # y = innovation
    y[1] = [pi_2_pi(y[1][0])]
    #print("predicted z pose: ", zp, " actual z pose: ", z, " innovation: ", y, " lmid: ", LMid + 1)

    H = jacobH(q, delta, xEst, LMid + 1, isFrozen)
    S = matrix_add(matrix_multiply(matrix_multiply(H, PEst), transpose_matrix(H)), [row[0:2] for row in Cx[0:2]])

    return y, S, H



def calc_innovation_pos(xEst, PEst, z):
    """
    Calculates the innovation based on expected position and observed system position

    :param obs:   observed system pos
    :param xEst: estimated position/state
    :param PEst: estimated covariance
    :param z:    observed system pos
    :param LMid: landmark id
    :returns:    returns the innovation y, and the jacobian H, and S, used to calculate the Kalman Gain
    """
    y = matrix_subtract(z, xEst[:3])
    y[2][0] = pi_2_pi(y[2][0])
    nLM = calc_n_LM(xEst)
    H = horizontal_stack(identity_matrix(3), zeros_matrix(3, 2 * nLM))
    S = matrix_add(matrix_multiply(matrix_multiply(H, PEst), transpose_matrix(H)), [row[0:3] for row in Cx[0:3]])

    return y, S, H

def jacobH(q, delta, x, i, isFrozen=False):
    """
    Calculates the jacobian of the measurement function

    :param q:     the range from the system pose to the landmark
    :param delta: the difference between a landmark position and the estimated system position
    :param x:     the state, including the estimated system position
    :param i:     landmark id + 1
    :returns:     the jacobian H
    """
    sq = math.sqrt(q)
    G = [[-sq * delta[0][0], - sq * delta[1][0], 0, sq * delta[0][0], sq * delta[1][0]],
         [delta[1][0], - delta[0][0], - q, - delta[1][0], delta[0][0]]]

    G = scalar_multiply(1/q, G)
    nLM = calc_n_LM(x)
    F1 = horizontal_stack(identity_matrix(3), zeros_matrix(3, 2 * nLM))

    if isFrozen:
        F2 = horizontal_stack(
            horizontal_stack(
                horizontal_stack(
                    zeros_matrix(2, 3),
                    zeros_matrix(2, 2 * (i - 1))
                ),
                zeros_matrix(2,2)
            ),
            zeros_matrix(2, 2 * nLM - 2 * i)
        )
    else:
        F2 = horizontal_stack(
            horizontal_stack(
                horizontal_stack(
                    zeros_matrix(2, 3),
                    zeros_matrix(2, 2 * (i - 1))
                ),
                identity_matrix(2)
            ),
            zeros_matrix(2, 2 * nLM - 2 * i)
        )

    F = vertical_stack(F1, F2)
    H = matrix_multiply(G, F)
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

    Fx = horizontal_stack(identity_matrix(STATE_SIZE),
                          zeros_matrix(STATE_SIZE, LM_SIZE * calc_n_LM(x)))

    jF = [[0.0, 0.0, -DT * u[0][0] * math.sin(x[2][0])],
          [0.0, 0.0, DT * u[0][0] * math.cos(x[2][0])],
          [0.0, 0.0, 0.0]]

    G = matrix_add(identity_matrix(STATE_SIZE),
                   matrix_multiply(matrix_multiply(transpose_matrix(Fx), jF), Fx))

    #if calc_n_LM(x) > 0:
    #    print(len(Fx), len(Fx[0])) # prints the shape of Fx

    return G, Fx,

def calc_LM_Pos(x, z):
    """
    Calculates the pose in the world coordinate frame of a landmark at the given measurement.

    :param x: [x; y; theta]
    :param z: [range; bearing]
    :returns: [x; y] for given measurement
    """
    zp = [[0], [0]]

    zp[0][0] = x[0][0] + z[0] * math.cos(x[2][0] + z[1])
    zp[1][0] = x[1][0] + z[0] * math.sin(x[2][0] + z[1])

    return zp


def calc_LM_Pos_Rp(x, z):
    """
    Calculates the pose in the world coordinate frame of a landmark given the relative xy position of the landmark

    :param x: [x; y; theta]
    :param z: [dx; dy]
    :returns: [x; y] for given measurement
    """
    zp = [[0], [0]]

    zp[0][0] = x[0][0] + z[0]
    zp[1][0] = x[1][0] + z[1]

    return zp


def get_LM_Pos_from_state(x, ind):
    """
    Returns the position of a given landmark

    :param x:   The state containing all landmark positions
    :param ind: landmark id
    :returns:   The position of the landmark
    """
    lm = x[STATE_SIZE + LM_SIZE * ind: STATE_SIZE + LM_SIZE * (ind + 1)]

    return lm

def set_LM_Pos_into_state(x, ind, lmpos):
    """
    Returns the position of a given landmark

    :param x:   The state containing all landmark positions
    :param ind: landmark id
    :returns:   Updated state
    """
    x[STATE_SIZE + LM_SIZE * ind: STATE_SIZE + LM_SIZE * (ind + 1)] = [[lmpos[0]], [lmpos[1]]]

    return x



def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

