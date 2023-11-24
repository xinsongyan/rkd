# RKD - Robot Kinematics and Dynamics
It is used to compute the kinematics and dynamics for robots.

## Dependencies


### [eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page)

you can install with apt-get:

`sudo apt-get install libeigen3-dev`


### [pinocchio](https://github.com/stack-of-tasks/pinocchio) & [eigenpy](https://github.com/stack-of-tasks/eigenpy)

you can install them from robotpkg apt repository:

1. Ensure you have some required installation dependencies
```
 sudo apt install -qqy lsb-release curl
 ```
2. Register the authentication certificate of robotpkg:
 ```
 sudo mkdir -p /etc/apt/keyrings
 curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | sudo tee /etc/apt/keyrings/robotpkg.asc
```
3. Add robotpkg as source repository to apt:
```
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list
```

4. Update the package index:
```
sudo apt update
```

5. Install pinocchio and eigenpy: (assumming you are using python3.8, adapt your desired python version accordingly)
```
 sudo apt install -qqy robotpkg-py38-pinocchio robotpkg-py38-eigenpy
```

6. configure your environment to use robotpkg:
   All the packages will be installed in the `/opt/openrobots` directory. To make use of installed libraries and programs, you must need to configure your PATH, PKG_CONFIG_PATH, PYTHONPATH and other similar environment variables to point inside this directory. For instance:
```
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH # Adapt your desired python version here
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
```

# Installation
Change to the project root directory and run the following commands:
```
mkdir build
cd build
cmake ..
make
sudo make install
```
By default, the tests is off, you can turn it on by adding `-DENABLE_TESTS=ON` to the cmake command.
