# p2o viewer

## Description

p2o viewer is a Qt + PCL based application for p2o.

## Requirements
- Eigen3
- Qt5
- PCL 1.14
- VTK9

## Build
### Install PCL 1.14 (Required for Ubuntu 22.04)
```bash
wget https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.14.1/source.tar.gz -O pcl.tar.gz
tar -xvf pcl.tar.gz
cd pcl
cmake -Bbuild -DCMAKE_INSTALL_PREFIX=/opt/pcl .
cmake --build build
sudo cmake --install build
```
### Build p2o viewer
```bash
git clone https://github.com/furo-org/p2o
cd p2o
CMAKE_PREFIX_PATH=/opt/pcl cmake -Bbuild . 
cmake --build build
```
### Run p2o viewer
```bash
./build/p2o_viewer
```
