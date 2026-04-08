"""
Kinematics Solvers
Both inverse and forward

Acknowledgements:
Jacobian math came from Sam's research
Forward Kinematics followed course notes provided by Romeo
"""

# from StewartPlatform import StewartPlatform
import numpy as np

def rpy2rot(rpy, deg = True):
    if deg:
        rpy = np.deg2rad(rpy)
    
    roll, pitch, yaw = rpy

    Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def rot2rpy(R, deg=True):    
    # cp = cos(pitch). If cp is 0, we are in gimbal lock.
    cp = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = cp < 1e-6

    if not singular:
        roll  = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], cp)
        yaw   = np.arctan2(R[1, 0], R[0, 0])
    else:
        # gimbal lock case
        pitch = np.arctan2(-R[2, 0], cp)
        roll  = 0.0  # set roll to zero
        if R[2, 0] < 0:  # Pitch is +90 deg
            yaw = np.arctan2(R[0, 1], R[1, 1])
        else:            # Pitch is -90 deg
            yaw = np.arctan2(-R[0, 1], R[1, 1])

    if deg:
        return np.degrees([roll, pitch, yaw])
    return np.array([roll, pitch, yaw])

def pre_rotate(rpy):
    pitch_offset = -90
    r, p, y = rpy
    return (r, p + pitch_offset, y)

def post_rotate(rpy):
    pitch_offset = 90
    r, p, y = rpy
    return (r, p + pitch_offset, y)

def solve_ik(X_base, X_goal, platform, deg = True):
        """
        Inputs:
        X_base: xyzrpy (m, m, m, deg, deg, deg)
        X_goal: xyzrpy (m, m, m, deg, deg, deg)
        deg: flag for whether or not rpy is deg or rad
    
        Output:
        lengths:         length of a leg
        lines:           start and end point of a leg
        base_coords:     point pos on base
        plat_coords:     point pos on plat
        """

        # extract
        base_pos, base_rpy = X_base[:3], X_base[3:]
        goal_pos, goal_rpy = X_goal[:3], X_goal[3:]

        # define leg connections
        leg_con_indicies = [(1, 0), (2, 1), (3, 2), (4, 3), (5, 4), (0, 5)]


        # get rotation matricies
        R_base = rpy2rot(base_rpy, deg=deg)
        R_goal = rpy2rot(goal_rpy, deg=deg)

        # find vector from base pos to goal pos, and goal vector
        T = (np.array(goal_pos) - np.array(base_pos)).reshape(3,1)
        B = np.array(base_pos).reshape(3, 1)

        # changing from goal to plat now because this is where the platform
        # has 'moved' to. It was the goal but now that we have the lengths
        # it is now where the platform is

        base_coords = B + (R_base @ platform.local_base_points)
        plat_coords = T + (R_goal @ platform.local_plat_points)

        lengths = np.zeros(6) # length of each leg
        lines = np.zeros((6, 2, 3)) # vector of each leg


        for i, (b_idx, p_idx) in enumerate(leg_con_indicies):
            base_point = base_coords[:, b_idx]
            plat_point = plat_coords[:, p_idx]
            leg_vec = plat_point - base_point
            lengths[i] = np.linalg.norm(leg_vec)
            lines[i] = (base_point, plat_point)

        return lengths, lines, base_coords, plat_coords


def jacobian(platform, X_base, X = [0, 0, 0, 0, 0, 0]):
    # define connections for base and plat points, leg 1 is point index 1 in base to point index 0 in plat
    leg_con_indices = [(1,0), (2,1), (3,2), (4,3), (5,4), (0,5)]

    _, _, B, _ = solve_ik(X_base, X, platform)

    t = np.array(X[0:3])
    Rot = rpy2rot(X[3:])
    # Euler-rate: base angle velocity matrix:
    # T = [[cpsi * ct, -spsi, 0,]
    #      [spsi * ct,  cpsi, -;]
    #      [  -st    ,  0,     1]]

    J = np.zeros((6, 6))
    for i, (b_idx, p_idx) in enumerate(leg_con_indices):
        pB = Rot @ platform.local_plat_points[:, p_idx]
        r = t + pB - B[:, b_idx]
        l = np.linalg.norm(r)
        u = r / l
        J[i, 0:3] = u.T
        J[i, 3:] = np.cross(pB, u).T
    return J

def solve_fk(platform, X_prev, d_goal):
    """
    Inputs:
    X_prev: xyz rpy of the previous state (platform)
    d_goal: ik lengths, these are the leg lengths we are trying to reach

    Outputs:
    X - Approximated Platform state
    ------------------------------------
    The forward kinematics solver works by taking two inputs:
    the pose of the platform in the previous state and the current
    leg lengths. Here they are denoted X_prev and d_goal respectively

    We then use Newton-Raphson approximation:
    1 d = IK(p, R)
    2 J = Jac(p, R)
    3 d x_ = J.inv()(d_goal - d_)
    4 p = p + dp
    R = dR * R

    5 IF ||d_goal - d|| < tolerance

    else go to 1


    NOTE: A pre-rotation needs to be applied to pitch, before calculating d_goal and passing X_prev to
    the forward kinematics solver. This is to bypass gimbal locking issue at 90deg pitch. This is because
    the ik i'm solving is to a distorted platform as i'm only rotating the plates, not translating them



    """

    tolerance = 0.00001
    X_current = np.array(X_prev)
    X_base = platform.X_base

    for _ in range(100):
        # should take no more than 3 iterations

        p = X_current[:3]
        rpy = X_current[3:]

        # leg length guess
        d_guess, _, _, _ = solve_ik(X_base, X_current, platform)

        if np.linalg.norm(d_goal - d_guess) < tolerance:
            break

        J = jacobian(platform, X_base, X_current)

        alpha = 0.5 # damping factor to prevent over rotation when updating rotation

        dx = alpha * np.linalg.pinv(J, rcond=1e-4) @ (d_goal - d_guess)

        dp = dx[:3]
        drpy = dx[3:]

        p = p + dp

        Rot = rpy2rot(rpy)
        dRot = rpy2rot(drpy, deg=False)

        Rot = dRot @ Rot

        rpy = rot2rpy(Rot)

        X_current = np.concatenate((p, rpy))


    # undo rotations -> only need to do it for the platform as
    # that's what we're outputting
    X_current[3:] = post_rotate(X_current[3:])

    # X_current is best guess of the current platform position
    return X_current

# def main():
#     np.set_printoptions(precision=3, suppress=True, linewidth=120)

#     base_r = 7
#     platform_r = 5
#     offset_angle = 5 
#     platform = StewartPlatform(base_r, platform_r, offset_deg=offset_angle)

#     # prerotate global frame so that yaw = 0 to prevent gimbal locking -> yaw at 90 induces gimbal lock
#     base_pos = [0, 0, 0]
#     base_rpy = pre_rotate([0, 90, 0])
#     X_base = np.concatenate((base_pos, base_rpy))

#     goal_pos = [10, 0, 2]
#     goal_rpy = pre_rotate([5, 90, 25])
#     X_goal = np.concatenate((goal_pos, goal_rpy))
    
#     prev_pos = [1, 0, 0]
#     prev_rpy = pre_rotate([2, 90, 0])
#     X_prev = np.concatenate((prev_pos, prev_rpy))


#     platform.set_Pose(X_base)

#     d_goal, _, _, _ = solve_ik(X_base, X_goal, platform)

#     print(solve_fk(platform, X_prev, d_goal))

# if __name__ == '__main__':
#     main()