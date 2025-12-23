# p2o2 (p2o v2): Petite Portable Pose-graph Optimizer v2

p2o2 is a compact and portable 3D pose-graph optimization library.

## Difference Between Previous Version
- Lie Algebra Optimization
- Customizable Error Functions
  - Relative 6DoF Pose (odometry, scan matching)
  - 3D Position (GNSS)
  - Attitude (for IMU)

Original p2o is still available and can be accessed in 'p2o_v1' tag.

## Requirements
p2o v2 is dependent on Eigen3 and C++14.

## Usage
```./sample_run_p2o <p2ofile>```

Options:
- max no. of iterations (-m) default: 300
- min no. of iterations (-n) default: 50
- robust threshold (-r) default: 0.01

## Run p2o with sample GNSS data
- ```cmake -Bbuild .```
- ```cmake --build build```
- ```./build/sample_run_p2o samples/data/cit_tsudanuma2020/CIT_2020_odom_gnss.p2o```
- ```cd samples/data/cit_tsudanuma2020/ && gnuplot CIT_2020_odom_gnss.plt #trajectories will be saved in png files```

## Extension to g2o File Format
p2o2 extends the g2o file format to support GNSS and Altitude constraints.

### EDGE_LIN3D
```EDGE_LIN3D id_from id_to dx dy dz  info00 info01 info02 info11 info12 info22```

```EDGE_LIN3D``` is an edge type for integrating 3D position observations into the pose graph.
Fuse relative position measurements (e.g., GNSS, motion capture, etc.) into the pose graph.

```math
\mathbf{e}_{\text{LIN3D}}
=
\mathbf{R}_a^\top (\mathbf{t}_b - \mathbf{t}_a) - \mathbf{z}
```


#### Typical Usage for GNSS ####
Add node 0 as a fixed vertex representing the Earth/world frame.
Add pose vertices for the robot states starting from node 1.

For each GNSS measurement, add an EDGE_LIN3D between node 0 and the corresponding pose node.
The measurement represents the observed position of the pose relative to the world origin.

### EDGE_GRAVITY
```EDGE_GRAVITY id_from id_to gx gy gz info00 info01 info11```

```EDGE_GRAVITY``` is an edge type for constraining the gravity direction of a pose.
The error between the measured gravity vector and the gravity vector estimated from the pose's orientation is evaluated by cross product and X, Y components are used for optimization.

```math
\mathbf{e}_{\text{GRAVITY}}
=
\mathbf{P}\mathbf{\hat{g}}\times \mathbf{g}
```
```math
\mathbf{P} = \begin{bmatrix}1 & 0 & 0 \\ 0 & 1 & 0 \\ 0& 0& 0 \end{bmatrix}
```

#### Typical IMU-based Attitude Integration ####
Add node 0 as a fixed and upright vertex.
Add pose vertices for the robot states starting from node 1.
For each attitude measurement, add an EDGE_GRAVITY between node 0 and the corresponding pose node.
gx, gy, gz represent the gravity vector in the local frame of the pose node.

