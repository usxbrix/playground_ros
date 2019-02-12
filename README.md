roslaunch playground_ros dnn_detect.launch camera:=/usb_cam image:=image_raw

v4l2-ctl --all
rosrun usb_cam usb_cam_node _framerate:=1 _pixel_format:=yuyv

## Object Tracker Pixy2
`rosrun playground_ros object_tracker_pixy2.py _track:=2 _min_rotation_speed:=2 _x_threshold:0.3 _rate:=20`