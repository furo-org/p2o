#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as R


def oplus_se3(pose_base, pose_rel):
    qbase    = R.from_quat(pose_base[3:])
    quat_rel = R.from_quat(pose_rel[3:])
    trans_rel = qbase.as_matrix() @ pose_rel[:3]
    new_orientation = qbase * quat_rel
    #print(new_orientation.as_rotvec())
    return np.concatenate([pose_base[:3] + trans_rel, new_orientation.as_quat()])


def ominus_se3(pose, pose_base):
    position_diff = pose[:3] - pose_base[:3]
    qbase = R.from_quat(pose_base[3:])
    quat  = R.from_quat(pose[3:])
    trans_diff = qbase.as_matrix().T @ position_diff
    orientation_diff = qbase.inv() * quat
    #print(orientation_diff.as_rotvec())
    return np.concatenate([trans_diff, orientation_diff.as_quat()])

