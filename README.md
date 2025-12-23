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

## Extension to [g2o](https://github.com/RainerKuemmerle/g2o) File Format
p2o2 uses g2o-style file format for defining pose graphs.
In addition to the standard g2o vertex and edge types for SE3 poses and relative pose constraints,
p2o2 intorduces the following new edge types.

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

Note that GNSS measurements cannot be used directly in latitude/longitude form.
They must first be converted into a local Cartesian coordinate system (e.g., ENU, UTM)
before being added to the pose graph.

#### Pseudo Measurements for Planar Correction ####
In some applications, the environment or map is expected to be locally planar (e.g., ground vehicles or 2.5D maps).
However, during optimization the pose graph may slowly warp in the vertical direction due to accumulated noise or weak constraints.
To mitigate this effect, ```EDGE_LIN3D``` can also be used as a pseudo measurement to softly stabilize poses onto a reference plane such as z = 0.

Set z component to the desired plane height (e.g., 0), and use current estimate for x and y.
Assign small information values to x and y components and assign a relatively large information value to the z component to enforce the planar constraint.
Below is an example of adding a pseudo measurement to node 99 to set its z coordinate to 0 with high confidence.
```
# sample pseudo measurement to constrain node 99 onto z=0 plane
VERTEX_SE3:QUAT 99 1.23 4.56 7.8 0 0 0 1
EDGE_LIN3D 0 99 1.23 4.56 0 1e-6 0 0 1e-6 0 1e-1
```

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

