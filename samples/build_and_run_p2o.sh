#!/bin/sh

# download data by Luca Carlone https://lucacarlone.mit.edu/datasets/

wget https://www.dropbox.com/s/vcz8cag7bo0zlaj/input_INTEL_g2o.g2o?dl=0 -O intel.g2o -nc
wget https://www.dropbox.com/s/d8fcn1jg1mebx8f/input_MITb_g2o.g2o?dl=0 -O mit_killian.g2o -nc
wget https://www.dropbox.com/s/gmdzo74b3tzvbrw/input_M3500_g2o.g2o?dl=0 -O manhattan3500.g2o -nc
wget https://www.dropbox.com/s/zu23p8d522qccor/parking-garage.g2o?dl=0 -O parking-garage.g2o -nc

# modified version of datasets

wget http://www.furo.org/irie/datasets/sphere2200_guess.g2o -nc
wget http://www.furo.org/irie/datasets/torus3d_guess.g2o -nc

# build p2o sample

cmake .
make

# run p2o sample

./test_p2o
gnuplot plot_data.plt

