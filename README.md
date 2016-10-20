# Stereo
### A Ros package for real-time stereo depth map and pointcloud generation.
### 
### Real Time Stereo Matching Video:
[![Alt text for your video](http://img.youtube.com/vi/ZUinHSjUZNM/0.jpg)](https://www.youtube.com/watch?v=ZUinHSjUZNM)

### Disparity Image Generation:
<img src="https://s15.postimg.org/4kwsa9ygr/stereo_pessoa.png" width="500"/>
### The disparity image is generated using stereoSGBM and WLSFilter in order to obtain a better edge definition of the disparity map. 



## stereo_node: 
### perform the depth image generation and pointcloud generation.
### subscribes: camera/left/ret and camera/right/ret. 
### both topics contain retified images, change for your own camera or use retify.py node to retify images
### publish in camera/depth/image and camera/depth/pointcloud

## rectify.py:
### rectify images based on params saved in stereo/params
### subscribe: /camera/left/image_raw and  /camera/right/image_raw
### change for your own camera.

## camera_simulador:
### simulate a camera loading and publishing images left.png and right.png from stereo/sample_images
### publish in /camera/left/image_raw and /camera/right/image_raw 

## depends:
### ROS (tested with kinetic)
### opencv 3.1 with contrib modules
### pcl

## To do:
### launch file


### Ricardo Achilles Filho
### rachillesf@gmail.com



