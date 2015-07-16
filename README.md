# ROAMFREE
The ROAMFREE sensor fusion library

# Dependencies

ROAMFREE relies on Eigen3, boost-python (>= 1.5) and on a modified 
version of g2o (included), which in turn relies on suitesparse. 
Furthermore, it requires a compiler which supports C 11, (e.g. gcc 4.7 or newer).

Under Ubuntu 12.10 you can provided the required dependencies
installing the following packages

- libsuitesparse-dev
- libboost-python1.50-dev
- libeigen3-dev

# Build

Two build types are available
-Debug
-Release

The release type is detected depending on the name of the folder 
in which you invoke cmake.

Debug release type is has **a lot** of debug options and screen output
enabled. Release instead is optimized for speed and it relies on an
included version of openmp to parallelize estimation tasks.

ROAMFREE uses cmake for makefile generation. ROAMFREE also supports catkin build.

`<src_path>` is the local copy of the ROAMFREE repository.
`<catkin_workspace>` is the catkin workspace you want to use.
`<build_path>` is the location where you want to build the sources.
`<install_path>` is the location where you want to install ROAMFREE.
`<build_type>` := (`Debug` | `Release`).


## Using catkin (ROS)

1) checkout the code in the src folder of a catkin workspace
2) cd <catkin_workspace>
3) catkin_make  -DCMAKE_BUILDTYPE=<build_type>


### installing with catkin (ROS)

1) checkout the code in the src folder of a catkin workspace
2) cd <catkin_workspace>
3) catkin_make_isolated --install [--install-space <install_path>]  -DCMAKE_BUILDTYPE=<build_type>


## - Using plain cmake

```
cd <build_path>
mkdir build
cd build
cmake <src_path>/roamfree -DCMAKE_BUILDTYPE=<build_type>
make
```

### - installing with plain cmake

after the build use the following command:

make install

TODO: not yet implemented

# Documentation

Documentation for ROAMFREE can be automatically generated with doxygen.
Please note that we inserted into the automatic doc generation script
only the classes which are meant to be employed from outside ROAMFREE.
However, comments are present in the code to understand the internal details.

```
cd <src_path>
doxygen Doxyfile
```

browse the documentation starting from the just generated 

```
<src_path>/doc/html/index.html
```