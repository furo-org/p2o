# p2o2 (p2o v2): Petite Portable Pose-graph Optimizer v2

p2o2 is a compact and portable 3D pose-graph optimization library.

## Difference Between Previous Version
- Lie Algebra Optimization
- Customizable Error Functions
  - Relative 6DoF Pose (odometry, scan matching)
  - 3D Position (GNSS)
  - Altitude (for IMU, coming soon)

Original p2o is still available and can be accessed in 'p2o_v1' tag.

## Requirements
p2o v2 is dependent on Eigen3 and C++14.

## Usage
```./sample_run_p2o <p2ofile>```

Options:
- max no. of iterations (-m) default: 300
- min no. of iterations (-n) default: 50
- robust threshold (-r) default: 0.01

