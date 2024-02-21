#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as R

# sample usage:
# p0 = np.array([1, 2, 3, 0, 0, np.sin(np.pi/4), np.cos(np.pi/4)])
# p1 = np.array([0.5, -1, 2.5, 0, 0, 0, 1])
# result = kgeom3d.oplus_se3(p0, p1)
# result: [2, 2.5, 5.5, 0, 0, 0.70710678, 0.70710678]


def oplus_se3(pose_base, pose_rel):
    """
    Perform relative pose addition (composition) in SE(3).

    Parameters:
    - pose_base: The base pose as a numpy array [x, y, z, qx, qy, qz, qw].
    - pose_rel: The relative pose to add, in the same format.

    Returns:
    - The resulting pose after adding pose_rel to pose_base.
    """
    qbase    = R.from_quat(pose_base[3:])
    quat_rel = R.from_quat(pose_rel[3:])
    trans_rel = qbase.as_matrix() @ pose_rel[:3]
    new_orientation = qbase * quat_rel
    #print(new_orientation.as_rotvec())
    return np.concatenate([pose_base[:3] + trans_rel, new_orientation.as_quat()])


def ominus_se3(pose, pose_base):
    """
    Compute the relative pose subtraction (difference) in SE(3).

    Parameters:
    - pose: The pose as a numpy array [x, y, z, qx, qy, qz, qw].
    - pose_base: The base pose to subtract from 'pose', in the same format.

    Returns:
    - The relative pose from pose_base to pose.
    """
    position_diff = pose[:3] - pose_base[:3]
    qbase = R.from_quat(pose_base[3:])
    quat  = R.from_quat(pose[3:])
    trans_diff = qbase.as_matrix().T @ position_diff
    orientation_diff = qbase.inv() * quat
    #print(orientation_diff.as_rotvec())
    return np.concatenate([trans_diff, orientation_diff.as_quat()])

