roslaunch playground_ros dnn_detect.launch camera:=/usb_cam image:=image_raw

v4l2-ctl --all
rosrun usb_cam usb_cam_node _framerate:=1 _pixel_format:=yuyv
