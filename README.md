# ROS_StereoWLS
A Ros node for stereo disparity generation using stereo SGBM and WLS filtering, with opencv.


<img src="https://s15.postimg.org/4kwsa9ygr/stereo_pessoa.png" width="500"/>
<img src="https://s13.postimg.org/63fyidton/stereo_cadeiras.png" width="500"/>
<img src="https://s22.postimg.org/8z2tsxixt/stereo_cadeira_mesa.png" width="500"/>

## stereo_node: 
subscribes: camera/left/ret and camera/right/ret
both topics contain retified images, change for your own camera or use retify.py node to retify images
publish in camera/depth

## retify.py:
subscribes: /camera/left/image_raw and  /camera/right/image_raw
change for your own camera.

## depends:
ROS (tested with kinetic)
opencv 3.1 with contrib modules

## To do:
make launch file that works


### Ricardo Achilles Filho
### rachillesf@gmail.com



