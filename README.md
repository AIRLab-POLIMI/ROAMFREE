![ROAMFREE](doc/images/roamfree02.jpg)

The ROAMFREE sensor fusion library allows to formulate and solve complex localization and sensor self-calibraion problems. It is based on a state-of-the-art pose-graph formulation and it relies on the popular [g2o](https://github.com/RainerKuemmerle/g2o) solver.

ROAMFREE is released under the LGPLv3 licence.

### Bibliography

For more information on the mathematical and algorithmic details you can refer to the [ICRA](http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=6907016) 
and [JOSER](http://joser.unibg.it/index.php?journal=joser&page=article&op=view&path%5B%5D=76) papers. In the context of photogrammetry, please refer to the [ISPRS journal paper](https://www.sciencedirect.com/science/article/abs/pii/S0924271617301387).


```
@article{cucci2017bundle,
  title={Bundle adjustment with raw inertial observations in UAV applications},
  author={Cucci, Davide Antonio and Rehak, Martin and Skaloud, Jan},
  journal={ISPRS Journal of photogrammetry and remote sensing},
  volume={130},
  pages={1--12},
  year={2017},
  publisher={Elsevier}
}

@inproceedings{cucci2014position,
  title={Position tracking and sensors self-calibration in autonomous mobile robots by Gauss-Newton optimization},
  author={Cucci, Davide Antonio and Matteucci, Matteo},
  booktitle={Robotics and Automation (ICRA), 2014 IEEE International Conference on},
  pages={1269--1275},
  year={2014},
  organization={IEEE}
}

@article{cucci2014development,
  title={On the Development of a Generic Multi-Sensor Fusion Framework for Robust Odometry Estimation},
  author={Cucci, Davide Antonio and Matteucci, Matteo},
  journal={Journal of Software Engineering for Robotics},
  volume={5},
  number={1},
  pages={48--62},
  year={2014}
}
```



# Dependencies

ROAMFREE relies on Eigen3, and on a modified 
version of g<sup>2</sup>o, which in turn relies on suitesparse. 
Furthermore, it requires a compiler which supports C++11, (e.g., gcc 4.7 or newer).

On Ubuntu you can provided the required dependencies
with

```sudo apt-get install libsuitesparse-dev libeigen3-dev libboost-all-dev```

# Build

ROAMFREE has been tested on Ubuntu 16.04, 18.04 and 20.04. Windows and Max OS X builds are most likely possible but untested.

## Standalone build (non-ROS)

The code base can be cloned and ROAMFREE can be built as follows:

```
git clone https://github.com/AIRLab-POLIMI/ROAMFREE.git
mkdir build
cd build
cmake ../roamfree/ -DCMAKE_BUILD_TYPE=<build type>
make
```

where `<build type>` has to be replaced with one of the two available options: `Debug` and `Release`. The `Debug` build has a number of integrity checks and screen output, and it is dramatically slower than the `Release` one. End user should always go for the latter.

Once ROAMFREE has been built, it is possible to install it as follows:

```
sudo make install
```

In case you want to change the instal path you can change the cmake command as
```
cmake ../roamfree/ -DCMAKE_INSTALL_PREFIX=<your_install_path>
```


## Catkin build (ROS)

In the following

- `<src_path>` is the local copy of the ROAMFREE repository,
- `<catkin_workspace>` is the catkin workspace you want to use,
- `<build_path>` is the location where you want to build the sources,
- `<install_path>` is the location where you want to install ROAMFREE,
- `<build_type>` is either `Debug` or `Release`.


ROAMFREE is composed of three ros packages, `roamfree`, `roamros` and `roamros_msgs`. 

As these folders contain ros packages, they have to live (or to be linked into) your catkin workspace.
If you do not have one, you can create it following [this tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

The recommended way is to link single package folders into the workspace, e.g.:

```
ln -s <src_path>/roamfree <catkin_workspace>/src/roamfree
```

then build with
```
cd <catkin_workspace>
catkin_make [--pkg roamfree] -DCMAKE_BUILD_TYPE=<build_type>
catkin_make [--pkg roamros] -DCMAKE_BUILD_TYPE=<build_type>
```

### Installing with catkin (optional)

```
cd <catkin_workspace>
catkin_make_isolated --install [--install-space <install_path>]  -DCMAKE_BUILD_TYPE=<build_type>
```


## Build the documentation (Doxygen)

Documentation for ROAMFREE can be automatically generated with doxygen.
Please note that the documentation is produced only for the public API of ROAMFREE. However, comments are present in the code to understand the internal details.

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

In case you have only one monitor you might get the following error, 
```
Attempted to access ss(2,1); index out of bounds because size(ss)=[1,4].

Error in configIMUGPSFusionTest (line 10)
config.global.figureOuterPosition = [ss(2,1) ss(2,2) ss(2,3) ss(2,4)]; % monitor 2
```
just comment out line 9 and comment line 10, there run again the config script. 

Then launch the viewer, again **in Matlab**, write

```
cd ../..
runViewer
```

you should see something like this:

![IMUGPSFusionTest]
(doc/images/IMUGPSFusionTestViewer.png)
