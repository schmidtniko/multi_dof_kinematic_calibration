# Note

We are in the process of publishing our software for multi-DOF kinematic calibration. Please give us a few more days...

ALL INFORMATION BELOW IS NOT FINAL

# Overview
This software allows the accurate calibration of kinematic hierarchies from sensor data. TODO

It uses AprilTags by Olson, that can simply be printed out and attached to walls or objects. To perform the reconstruction, you need a calibrated camera with a fixed focal length (no auto focus). The camera needs to be calibrated using the typical OpenCV camera calibration model. 

# Installation

## Dependencies

* [Ceres Solver](http://ceres-solver.org/)
* [OpenCV](http://opencv.org/)
* [Eigen 3.0](http://eigen.tuxfamily.org/)
* [Visual Marker Mapping Tool](https://github.com/cfneuhaus/visual_marker_mapping)

In Ubuntu, the first three dependencies can be installed using the command

* `apt-get install libceres-dev libsuitesparse-dev libopencv-dev`

In Arch Linux, use:

* `pacman -S eigen opencv`

* Ceres is available as an AUR package called [ceres-solver](https://aur.archlinux.org/packages/ceres-solver/).

The AprilTags dependency is automatically pulled in as a git submodule.

## Cloning

Via HTTPS:

`git clone --recursive https://github.com/cfneuhaus/multi_dof_kinematic_calibration.git`

Via SSH:

`git clone --recursive git@github.com:cfneuhaus/multi_dof_kinematic_calibration.git`

## Building

You need at least
* [CMake 3.0](https://cmake.org/)
* GCC 4.7 (?)

If you are using Ubuntu, this means that you need at least Ubuntu 14.04.

### Linux/Mac

```
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j5
```

### Windows

It should be possible to build our software on Windows, given that we do not use any platform specific features, but so far we have not attempted to build it there. Please let us know if you run into problems doing this.

# Usage

## Preliminaries

Our software works on *project paths*. A project path initially has to have the following layout:

```
my_project/.
my_project/camera_intrinsics.json
my_project/images/
my_project/images/your_image_1.jpg
my_project/images/anotherimage.png
...
```

* The *images* folder is supposed to contain all images that you want to use for calibration. Currently, all png and jpg files within the folder are being used. Note that they all have to have the same size, and they all have to correspond to the camera intrinsics specified in the *camera_intrinsics.json* file.
* The camera_intrinsics.json file is something you have to create before mapping (it is not required for detection only). See the [File Formats](#file-formats) section on how to create this one.
* Results of our tools are automatically written to the root of the project path. For example the marker detection writes a file called "marker_detections.json" to the root. The reconstruction result file is called "reconstruction.json".

## Running

Our software consists of a command-line tool called *multi_dof_kinematic_calibration* located in the *build/bin* folder, and a python script that can optionally be used to visualize the results in 3D.

multi_dof_kinematic_calibration:
* `--help`: Shows a help text.
* `--project-path`: Path to the aforementioned project directory.
* `--marker_width`, `--marker-height`: The marker width/height in meters. This is a marker size that is written to the *marker_detections.json* file with every marker. It is not used in any other way by the detection right now, but we feel that this information is an essential part of the detection result, which is why we include it. The marker can be configured to be slightly non-square, which can be used to compensate for bad and slightly distorted print-outs. If you have markers with different sizes, you will have to edit the *marker_detections.json* file by hand. If you do not care about the metrical size if your reconstruction, you can simply set both the width/height to any value, say *0.1*, or simply stick to the default value.

For visualization of the results in 3D, we also include a Python (2.7/3.0) script called "visualize_reconstruction.py". It is based on *pygame*, *OpenGL*, *GLU*, *GLUT*, *numpy*, and you may need to install the corresponding Python packages for your distribution in order to be able to run it.

The tool's only parameter is the path of the reconstruction.json file, that is being written by the visual_marker_mapping tool upon completion. The camera can be controlled using W, S, A, D. The mouse can be used to look around by holding the left mouse button. The camera speed can be increased by holding space.

## Example

We provide a test dataset, that you can use to test our tools. It is available [here](TODO).

Use the following steps to perform the marker detection and 3D reconstruction (assuming you are in the root folder of this repository):

```
wget TODO
unzip ptu_d47_w_asus_xtion_ir.zip
./build/bin/visual_marker_detection --project_path ptu_d47_w_asus_xtion_ir/cam1/
./build/bin/multi_dof_kinematic_calibration --project_path ptu_d47_w_asus_xtion_ir
```

### Output:

```
Solving full optimization problem...
    Full optimization returned termination type 0
    Full training reprojection error RMS: 0.815625 px
Solving full optimization problem...done!

Resulting Parameters:
    Tick2Rad for all joints:
        joint_0:0.000895964, joint_1:0.000918792, 
    Joint poses:
        joint_0:     1.10333           0           0    0.781601  0.00769075 -0.00254335   -0.623726
        joint_1:  -1.29697         0         0  -0.43817  0.557305 -0.439311  0.551747
        cam_joint_1:  0.00776045   0.0592888 -0.00959563    0.490016   -0.505044    0.489606    -0.51488
Test Reprojection Error RMS: 2.50235 px
Test Reprojection Error RMS for camera 1: 2.50235 px
```

TODO

If you want to visualize the results, simply run:
```
python3 visualize_reconstruction.py calibration_room1/reconstruction.json
```


# File Formats

TODO

Occurring rotations are represented as a unit quaternion in the order *w, x, y, z*. Rotation and translation together define a pose that transforms points from marker/camera space to world space. The local coordinate systems are defined as follows:
* When looking at a marker, the *x*-axis goes to the right, *y* up, and *z* points out of the marker plane.
* A cameras *x*-axis points to the right, *y*-axis down, and the *z*-axis in viewing direction.

# Authors

* Frank Neuhaus (fneuhaus_AT_uni-koblenz.de)
* Stephan Manthe

# Copyright and License

[GPL 3.0](http://www.gnu.org/licenses/gpl-3.0.en.html)

# Citing

If you use our work, please cite us:

TODO

# References

Ed Olson, [AprilTag: A robust and flexible visual fiducial system](http://april.eecs.umich.edu/papers/details.php?name=olson2011tags), Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), 2011
