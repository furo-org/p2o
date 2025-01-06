#!/usr/bin/env python3
# Copyright (C) 2024-2025 Kiyoshi Irie
# Generate p2o from rosbag file

import rosbag
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt
import sys
import os
import kgeom3d
from pyproj import Transformer

# parameters

# information matrix (upper triangle) for odometry observations
odom_infom = '1e2 0 0 0 0 0 1e2 0 0 0 0 1e2 0 0 0 1e2 0 0 1e2 0 1e2'
gnss_cov_thre = 0.001

def save_pcd(filename, points):
    with open(filename, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")
        for point in points:
            f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")


def latlon_to_xyz(trans, lat, lon, alt):
    x, y = trans.transform(lat, lon)
    return x, y, alt


args = sys.argv
assert len(args)>=2, "you must specify a rosbag file"

# get path
filename=os.path.normpath(os.path.join(os.getcwd(),args[1]))

# output path
output_basename = os.path.splitext(os.path.basename(filename))[0]
output_filename = output_basename + ".p2o"

output_dir = output_basename
if not os.path.isdir(output_dir):
    os.mkdir(output_dir)

output_path = os.path.join(output_dir, output_filename)

# Sample convert to Japan Plane Rectangular Coordinate System No. 6
transformer = Transformer.from_crs("epsg:4326", 'epsg:6674')

# read the bag file

bag = rosbag.Bag(filename)
prev_gnss_t = 0
vertices=[]
edges=[]
np_poses=None
np_gnss_list=np.zeros((bag.get_message_count('/fix'),8), dtype=np.float64)
gnss_cnt = 0
#np_gnss_list=None
id = 0
cloud_ind = 0
for topic, msg, t in bag.read_messages():
    t_since_epoch = t.secs + t.nsecs * 1e-9
    if topic=="/Odometry":
        id += 1
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
        if np_poses is None:
            np_poses=np_pose
        else:
            np_poses=np.append(np_poses,np_pose,axis=0)
        q = pose.orientation
        qstr = f'{q.x} {q.y} {q.z} {q.w}'
        cloud_file = f'cloud_{id:06d}.pcd'
        vertices.append(f'VERTEX_SE3:QUAT {id} {pose.position.x} {pose.position.y} {pose.position.z} {qstr} {cloud_file}')

    if topic=="/cloud_registered_body":
        cloud_ind += 1
        pc = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(list(pc))
        filename = os.path.join(output_dir, f'cloud_{cloud_ind:06d}.pcd')
        save_pcd(filename, points)

    if topic=="/fix":
        if id > 0:
            if msg.position_covariance[0] < gnss_cov_thre and (t_since_epoch - prev_gnss_t > 3):
                #print(f'gnss status: {msg.status.status}', file=sys.stderr)
                x, y, z = latlon_to_xyz(transformer, msg.latitude, msg.longitude, msg.altitude)
                np_gnss_list[gnss_cnt,0]=t_since_epoch
                np_gnss_list[gnss_cnt,1]=id
                np_gnss_list[gnss_cnt,2]=x
                np_gnss_list[gnss_cnt,3]=y
                np_gnss_list[gnss_cnt,4]=z
                np_gnss_list[gnss_cnt,5]=msg.position_covariance[0]
                np_gnss_list[gnss_cnt,6]=msg.position_covariance[4]
                np_gnss_list[gnss_cnt,7]=msg.position_covariance[8]
                prev_gnss_t = t_since_epoch
                gnss_cnt += 1

bag.close()

np_gnss_list = np_gnss_list[0:gnss_cnt,:]
mean_gnss = np.mean(np_gnss_list, axis=0)
#mean_gnss = np.zeros(8)

#print(mean_gnss, file=sys.stderr)

vertices.insert(0, f'VERTEX_SE3:QUAT 0 {mean_gnss[3]} {mean_gnss[2]} {mean_gnss[4]} 0 0 0 1')

for i in range(1, len(np_poses)):
    p = np_poses[i,:]
    prevp = np_poses[i-1,:]
    dp = kgeom3d.ominus_se3(p[1:],prevp[1:])
    edges.append(f'EDGE_SE3:QUAT {i} {i+1} {dp[0]} {dp[1]} {dp[2]} {dp[3]} {dp[4]} {dp[5]} {dp[6]} {odom_infom}')

for i in range(gnss_cnt):
    id = int(np_gnss_list[i,1])
    x  = np_gnss_list[i,2]-mean_gnss[2]
    y  = np_gnss_list[i,3]-mean_gnss[3]
    z  = np_gnss_list[i,4]-mean_gnss[4]
    xinfo = min(1.0, 1.0/np_gnss_list[i,5])
    yinfo = min(1.0, 1.0/np_gnss_list[i,6])
    zinfo = min(1e-3, 1.0/np_gnss_list[i,7])
    gnss_infom = f'{yinfo} 0 0 {xinfo} 0 {zinfo}'

    edges.append(f'EDGE_LIN3D 0 {id} {y} {x} {z} {gnss_infom}')

with open(output_path, 'w') as f:
    for v in vertices:
        f.write(v + '\n')
    for e in edges:
        f.write(e + '\n')

