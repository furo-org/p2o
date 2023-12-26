#!/usr/bin/env python3
# Generate p2o from rosbag file
# Usage: python3 sample_p2o_from_rosbag.py tsudanuma20_cit_compressed.bag > output.p2o

import rosbag
import numpy as np
import matplotlib.pyplot as plt
import sys
import os
from tqdm import tqdm
import kgeom3d
from pyproj import Transformer

# parameters

# information matrix (upper triangle) for odometry observations
odom_infom = '100 0 0 0 0 0 100 0 0 0 0 100 0 0 0 100 0 0 100 0 100'
gnss_cov_thre = 10


def latlon_to_xyz(trans, lat, lon, alt):
    x, y = trans.transform(lat, lon)
    return x, y, alt


args = sys.argv
assert len(args)>=2, "you must specify a rosbag file"

# get path
filename=os.path.normpath(os.path.join(os.getcwd(),args[1]))

# Sample convert to Japan Plane Rectangular Coordinate System No. 9
transformer = Transformer.from_crs("epsg:4326", 'epsg:6677')

# read the bag file

bag = rosbag.Bag(filename)
start_time = bag.get_start_time()
end_time = bag.get_end_time()
duration = end_time - start_time

prev_odom_pos = None
prev_gnss_t = 0
vertices=[]
edges=[]
np_poses=None
np_gnss_list=None
id = 0

with tqdm(total=duration, unit='sec', desc='Processing messages') as pbar:
    for topic, msg, t in bag.read_messages():
        t_since_epoch = t.secs + t.nsecs * 1e-9
        if topic=="/icart_mini/odom":
            pose = msg.pose.pose
            np_pose=np.zeros((1,8), dtype=np.float64)
            np_pose[0,0]=t_since_epoch
            np_pose[0,1]=pose.position.x
            np_pose[0,2]=pose.position.y
            np_pose[0,3]=pose.position.z
            np_pose[0,4]=pose.orientation.x
            np_pose[0,5]=pose.orientation.y
            np_pose[0,6]=pose.orientation.z
            np_pose[0,7]=pose.orientation.w

            if (prev_odom_pos is not None) and np.linalg.norm(np_pose[0,1:4] - prev_odom_pos[0,1:4]) < 0.1:
                continue

            if np_poses is None:
                np_poses=np_pose
            else:
                np_poses=np.append(np_poses,np_pose,axis=0)
            q = pose.orientation
            qstr = f'{q.x} {q.y} {q.z} {q.w}'
            id += 1
            vertices.append(f'VERTEX_SE3:QUAT {id} {pose.position.x} {pose.position.y} {pose.position.z} {qstr}')
            prev_odom_pos = np_pose
            current_time = t_since_epoch - start_time
            pbar.update(min(current_time - pbar.n, duration - pbar.n))

        if topic=="/fix":
            if msg.status.status == 0 and id > 0:
                if (np_gnss_list is not None) and id == int(np_gnss_list[-1,1]):
                    continue
                if msg.position_covariance[0] < gnss_cov_thre and (t_since_epoch - prev_gnss_t > 3):
                    #print(f'gnss status: {msg.status.status}', file=sys.stderr)
                    x, y, z = latlon_to_xyz(transformer, msg.latitude, msg.longitude, msg.altitude)
                    np_gnss=np.zeros((1,8), dtype=np.float64)
                    np_gnss[0,0]=t_since_epoch
                    np_gnss[0,1]=id
                    np_gnss[0,2]=x
                    np_gnss[0,3]=y
                    np_gnss[0,4]=z
                    np_gnss[0,5]=msg.position_covariance[0]
                    np_gnss[0,6]=msg.position_covariance[4]
                    np_gnss[0,7]=msg.position_covariance[8]
                    if np_gnss_list is None:
                        np_gnss_list = np_gnss
                    else:
                        np_gnss_list=np.append(np_gnss_list,np_gnss,axis=0)
                    prev_gnss_t = t_since_epoch

bag.close()


mean_gnss = np.mean(np_gnss_list, axis=0)

vertices.insert(0, f'VERTEX_SE3:QUAT 0 {mean_gnss[2]} {mean_gnss[3]} {mean_gnss[4]} 0 0 0 1')

for i in range(1, len(np_poses)):
    p = np_poses[i,:]
    prevp = np_poses[i-1,:]
    dp = kgeom3d.ominus_se3(p[1:],prevp[1:])
    edges.append(f'EDGE_SE3:QUAT {i} {i+1} {dp[0]} {dp[1]} {dp[2]} {dp[3]} {dp[4]} {dp[5]} {dp[6]} {odom_infom}')

for i in range(len(np_gnss_list)):
    id = int(np_gnss_list[i,1])
    x  = np_gnss_list[i,2]-mean_gnss[2]
    y  = np_gnss_list[i,3]-mean_gnss[3]
    z  = np_gnss_list[i,4]-mean_gnss[4]
    xinfo = min(1.0, 1.0/np_gnss_list[i,5])
    yinfo = min(1.0, 1.0/np_gnss_list[i,6])
    zinfo = min(1.0, 1.0/np_gnss_list[i,7])
    gnss_infom = f'{xinfo} 0 0 {yinfo} 0 {zinfo}'

    edges.append(f'EDGE_LIN3D 0 {id} {y} {x} {z} {gnss_infom}')

for v in vertices:
    print(v)

for e in edges:
    print(e)

