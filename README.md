# zed_jetson_interface
a simple interface between a ZED camera Jetson tx1. Assumes we have pcl and ZED api installed - and all the libraries in the cmake lists file. Captures a colored point cloud of the camera caputre every 2-3 seconds and saves it.

instruction to build with cmake:
$ mkdir build && cd build && cmake .. && make

to run, in  build directory:
$ ./collect


when we run the collect program, the program will collect the campera capture in the form of a colored point cloud 

