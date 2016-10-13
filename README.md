# ROS_StereoWLS
A Ros node for stereo disparity generation using stereo SGBM and WLS filtering, with opencv.
<img src="https://s18.postimg.org/48ufg1y89/Whats_App_Image_2016_10_12_at_16_37_10.jpg" width="500"/>

## stereo_node: 
subscribes: camera/left/ret and camera/right/ret
both topics contain retified images, change for your own camera or use retify.py node to retify images

## retify.py:
subscribes: /camera/left/image_raw and  /camera/right/image_raw
change for your own camera.

## To do:
make launch file that works


### Ricardo Achilles Filho
### rachillesf@gmail.com



