# StereoMagic
### A Ros package for real-time stereo depth map and pointcloud generation.
###
### Mobile Robot Real Time Stereo Matching Video :
#### (click to open)
[![Alt text for your video](http://img.youtube.com/vi/ZUinHSjUZNM/0.jpg)](https://www.youtube.com/watch?v=ZUinHSjUZNM)

### Disparity Image and PointCloud Generation:
<img src="https://s17.postimg.org/sz9l3om67/imagem_pointcloud2.png" width="800"/>

## Quick Example(no camera needed! uses sample images):
### $ roslaunch roslaunch stereo stereo_sim.launch
#### (please, change the GLOBAL_PATH variable in rectify.py and camera_simulator.py to the path of your workspace)


## Use it on Your Own camera feed:
### $ rosrun stereo stereo_node
###
### rqt_graph vis:
<img src="https://s13.postimg.org/70otej1av/node_graph.png" width="700"/>



## Dependences:
#### ROS (tested with kinetic)
#### opencv 3.1 with contrib modules
#### PCL 1.7 (comes with ROS kinect full-desktop-install)

## Hardware used in tests:
#### Pioneer P3-DX (for moving shoots and video recording)
#### PointGrey Bumblebee2 Camera
#### Dell Precision Workstation

## TODO:
#### improve rectify node using c++ and better calibration routine
####


### Ricardo Achilles Filho
### rachillesf@gmail.com
