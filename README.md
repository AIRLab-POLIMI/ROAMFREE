![ROAMFREE]
(https://github.com/AIRLab-POLIMI/ROAMFREE/blob/master/doc/images/roamfree02.jpg)

The ROAMFREE sensor fusion library

# Dependencies

ROAMFREE relies on Eigen3, and on a modified 
version of g2o, which in turn relies on suitesparse. 
Furthermore, it requires a compiler which supports C 11, (e.g. gcc 4.7 or newer).

Under Ubuntu 15.04 you can provided the required dependencies
with

```sodo apt-get install libsuitesparse-dev libeigen3-dev```

# Build

Currently, we suggest to build ROAMFREE in a ROS workspace, with `catkin_make`. Standalone build is also possible but it still requires some user intervention.

Two build types are available: `Debug` and `Release`.

`Debug` build should be avoided unlsee you are a developer. It has a number of integrity checks and screen output, and it is dramatically slower. End user should go for the `Release` build.

In the following

`<src_path>` is the local copy of the ROAMFREE repository,
`<catkin_workspace>` is the catkin workspace you want to use,
`<build_path>` is the location where you want to build the sources,
`<install_path>` is the location where you want to install ROAMFREE,
`<build_type>` is either `Debug` or `Release`.


## Catkin build (ROS)

ROAMFREE is composed of three ros packages, `roamfree`, `roamros` and `roamros_msgs`. 

You can either checkout the whole git repository into `<catkin_workspace>/src` or place there symbolic to the package folders, e.g.:

```
ln -s <src_path>/roamfree <catkin_workspace>/src/roamfree
```

then build with
```
cd <catkin_workspace>
catkin_make -DCMAKE_BUILDTYPE=<build_type>
```

### Installing with catkin (optional)

```
cd <catkin_workspace>
catkin_make_isolated --install [--install-space <install_path>]  -DCMAKE_BUILDTYPE=<build_type>
```

TODO: verify

## Standalone build (non-ROS)

TODO.

## Build the documentation (Doxygen)

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

# Run the examples

Some examples are included in the `ROAMtest` library to illustrate the basic usage of the library.

You may for instance run the `IMUGPSFusionTest`, which performs GPS+Inertial real time estimation from synthetic data.

Here we assume that you have built ROAMFREE as a ROS package.

```
rosrun roamfree IMUGPSFusionTest
```

By default, low level logs are generated in the folder `/tmp/roamfree`. Tese logs can be plot in real-time with the provided Matlab viewer. To do so first load the provided viewer configuration, specific for the test you are going to run. In this case **in Matlab** run

```
cd <src_dir>/_development/Matlab/PluginViewer/configs/ROAMtest
configIMUGPSFusionTest
```

then launch the viewer

```
cd ../..
runViewer
```

![ROAMFREE]
(https://github.com/AIRLab-POLIMI/ROAMFREE/blob/master/doc/images/IMUGPSFusionTestViewer.png)


