# ROS_StereoWLS
### A Ros node for real-time stereo disparity generation using stereo SGBM and WLS filtering, with opencv.

### Sample Image:
<img src="https://s15.postimg.org/4kwsa9ygr/stereo_pessoa.png" width="500"/>

### Video:
[![Alt text for your video](http://img.youtube.com/vi/ZUinHSjUZNM/0.jpg)](https://www.youtube.com/watch?v=ZUinHSjUZNM)

## stereo_node: 
### subscribes: camera/left/ret and camera/right/ret. 
### both topics contain retified images, change for your own camera or use retify.py node to retify images
### publish in camera/depth

## retify.py:
### subscribes: /camera/left/image_raw and  /camera/right/image_raw
### change for your own camera.

## depends:
### ROS (tested with kinetic)
### opencv 3.1 with contrib modules

## To do:
### make launch file that works


### Ricardo Achilles Filho
### rachillesf@gmail.com



