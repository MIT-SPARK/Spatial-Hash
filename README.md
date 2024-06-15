# Spatial Hash
A minimal library for spatial data structures based on voxel hashing. Depends only on `Eigen` and `glog` pulled in as system dependencies. 

## Table of contents
- [Credits](#credits)
- [Installation](#installation)

## Credits
This library was inspired by data structures used in [voxblox](https://github.com/ethz-asl/voxblox).
It was developed by [Lukas Schmid](https://schmluk.github.io/) at the [MIT-SPARK Lab](http://mit.edu/sparklab) and is released under a [BSD-3-Clause License](LICENSE)! Additional contributions welcome! This work was supported in part by the Swiss National Science Foundation and Amazon.

## Installation

Install system deps:
```
sudo apt install libeigen3-dev libgoogle-glog-dev libgtest-dev
```
Alternatively, if building with catkin or ros,
```
rosdep install --from-paths . --ignore-src -r -y
```
will install all required dependencies (from either the `src` directory of the workspace or the repo directory itself).

Clone repository:
```
cd ~/catkin_ws/src
git clone git@github.mit.edu:SPARK/Spatial-Hash.git spatial_hash
```

Option 1: Install via catkin:
```
catkin build spatial_hash
```

Option 2: Install via cmake:
```
cd spatial_hash
mkdir build
cd build
cmake ..
make -j

# optionally install this package
sudo make install
```

Setup pre-commit for contributing:
```
pip install pre-commit
pre-commit install
```
