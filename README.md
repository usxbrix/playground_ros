roslaunch playground_ros dnn_detect.launch camera:=/usb_cam image:=image_raw

v4l2-ctl --all
rosrun usb_cam usb_cam_node _framerate:=1 _pixel_format:=yuyv

## Object Tracker Pixy2
`rosrun playground_ros object_tracker_pixy2.py _track:=2 _min_rotation_speed:=2 _x_threshold:0.3 _rate:=20`
`rosrun playground_ros object_tracker_pixy2.py _track:=2 _min_rotation_speed:=1.8 _max_rotation_speed:5 _rate:=10 _x_threshold:=0.3`
`rosrun playground_ros object_tracker_pixy2.py _track:=2 _min_rotation_speed:=1.8 _max_rotation_speed:5 _rate:=20 _x_threshold:=0.25 _search_delay:=3 _ring_buffer_size:=1`

`rosrun playground_ros object_tracker_pixy2_follow.py _track:=2 _min_rotation_speed:=1.8 _max_rotation_speed:5 _rate:=20 _x_threshold:=0.5 _search_delay:=3 _ring_buffer_size:=1 _min_linear_speed:=0.15 _max_linear_speed:=0.364 _z_threshold:=0.02 _min_z:=0.02 _goal_z:=0.2`
