# Edge-aware-3D-reconstruction
An edge aware 3D reconstruction pipeline.

![alt text](https://github.com/JessicaKANG/Edge-aware-3D-reconstruction/blob/master/simpip.png)

In this project we present a novel Structure from Motion (SfM) algorithm which extends the generic pipeline with two more stages, reconstruction of 3D edge points and edge aware bundle adjustment. The idea of these two stages is to explore the directionality embedded in edge feature and constrain 3D edge points to move according to their edge direction during bundle adjustment (BA).

Requirements
============

The system was tested on **Ubuntu 16.04 LTS** with the following versions:

* CMake 
* OpenCV v2.4.13
* Eigen v3.3.1
* g2o c++03 branch
* EdgeGraph3D 

For environmental configuration, please refer to [here](https://jessicakang.github.io/blog/2018/04/08/%E8%A7%86%E8%A7%89%E7%8E%AF%E5%A2%83%E6%90%AD%E5%BB%BA/#more) and 
[here](https://jessicakang.github.io/blog/2018/04/13/%E8%A7%86%E8%A7%89%E7%8E%AF%E5%A2%83%E6%8B%86%E8%BF%81/).

Building
========

On Unix platforms EdgeGraph3D can be built by running the following commands from its containing folder:

    mkdir build
    cd build
    cmake -DOpenMVG_DIR=<path to OpenMVG cmake file> ..
    make
    
Usage
=======

An example showing how to use the system has been provided in OpenEDGE_pipeline.py.
