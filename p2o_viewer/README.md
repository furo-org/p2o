# p2o viewer

## Description

p2o viewer is a Qt + PCL based application for p2o.

## Requirements
- Eigen3
- Qt5
- PCL 1.14
- VTK >= 8

## Build
### Install Latest CMake
- see https://apt.kitware.com/

### Install VTK 8.2 (Required for Ubuntu 20.04)
```bash
wget https://www.vtk.org/files/release/8.2/VTK-8.2.0.tar.gz
tar -xvf VTK-8.2.0.tar.gz
cd VTK-8.2.0
cmake -DCMAKE_BUILD_TYPE=Release -DVTK_Group_Qt=ON -DCMAKE_INSTALL_PREFIX=/opt/vtk8 -Bbuild .
cmake --build build/
sudo cmake --install build
export CMAKE_PREFIX_PATH=/opt/vtk8
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/vtk8/lib
```

### Install PCL 1.14 (Required for Ubuntu 20.04/22.04)
```bash
wget https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.14.1/source.tar.gz -O pcl.tar.gz
tar -xvf pcl.tar.gz
cd pcl
cmake -Bbuild -DCMAKE_INSTALL_PREFIX=/opt/pcl .
cmake --build build
sudo cmake --install build
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/opt/pcl
```

### Build p2o viewer
```bash
git clone https://github.com/furo-org/p2o
cd p2o
cmake -Bbuild .               # if you built VTK or PCL from source, set CMAKE_PREFIX_PATH
cmake --build build
```

### Run p2o viewer
```bash
./build/p2o_viewer
```
