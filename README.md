# Stereo
### A Ros package for real-time stereo depth map and pointcloud generation.
### 
### Mobile Robot Real Time Stereo Matching Video :
#### (click to open)
[![Alt text for your video](http://img.youtube.com/vi/ZUinHSjUZNM/0.jpg)](https://www.youtube.com/watch?v=ZUinHSjUZNM)

### Disparity Image and PointCloud Generation:
<img src="https://s18.postimg.org/xfvf89nrt/imagem_pointcloud2.png" width="700"/>

## Quick Example(no camera needed!):
### $ roslaunch roslaunch stereo stereo_sim.launch 
#### (please, check the path of calibration files on rectify.py and camera_simulator.py,change it to your own global path)
#### This will load sample images and generate point cloud from them.

## Use on Your Own camera feed:
### $ rosrun stereo stereo_node
###
### rqt_graph vis:
<img src="https://s13.postimg.org/70otej1av/node_graph.png" width="700"/>



## Depends:
#### ROS (tested with kinetic)
#### opencv 3.1 with contrib modules
#### pcl

## Hardware used in tests:
#### Pioneer P3-DX (for moving shoots and video recording)
#### PointGrey Bumblebee2 Camera
#### Dell Precision Workstation

## TODO:
#### fix launch files path
#### improve rectify node using c++ and better calibration
#### 

### Ricardo Achilles Filho
### rachillesf@gmail.com



