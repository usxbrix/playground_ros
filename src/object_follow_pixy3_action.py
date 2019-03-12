#!/usr/bin/env python

"""
    object_tracker_pixy2.py - Version 2.0 2019 

    based on:
    
    object_tracker.py - Version 1.1 2013-12-20
    
    Rotate the robot left or right to follow a target published on the /roi topic.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
from sensor_msgs.msg import RegionOfInterest, CameraInfo, Range
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int8
from math import copysign, isnan

# from dnn_detect.msg import DetectedObject, DetectedObjectArray
from pixy2_msgs.msg import PixyBlock, PixyData, PixyResolution

import thread
import collections

import actionlib
import playground_ros.msg

class FollowAction(object):
    # create messages that are used to publish feedback/result
    _feedback = playground_ros.msg.FollowFeedback()
    _result = playground_ros.msg.FollowResult()

    # Intialize the movement command
    move_cmd = Twist()

    # Initialize the global ROI
    roi = RegionOfInterest()

    # Initialize the global PixyData
    pixydata = PixyData()

    # We will get the image width and height from the camera_info topic
    image_width = 0
    image_height = 0
        
    # Set flag to indicate when the ROI stops updating
    target_visible = False
        
    # Timestamp for last target
    last_target_time = rospy.Time()

    # Range
    range = 0.0

    # Get a lock for updating the self.move_cmd values
    lock = thread.allocate_lock()

    def __init__(self, name):
        self.load_params()
        self._action_name = name
        #self._as = actionlib.SimpleActionServer(self._action_name, actionlib_tutorials.msg.FibonacciAction, execute_cb=self.execute_cb, auto_start = False)
        self._as = actionlib.SimpleActionServer(self._action_name, playground_ros.msg.FollowAction, execute_cb=self.execute_cb_follow, auto_start = False)
        self._as.start()
        rospy.loginfo('SimpleActionServer started')

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)

        # Publisher to control the robot's movement
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Wait for the camera_info topic to become available
        # rospy.loginfo("Waiting for camera_info topic...")
        rospy.loginfo("Waiting for pixy resolution topic...")
        #rospy.wait_for_message('usb_cam/camera_info', CameraInfo)
        rospy.wait_for_message('pixy2_resolution', PixyResolution)

        # Subscribe the camera_info topic to get the image width and height
        #rospy.Subscriber('usb_cam/camera_info', CameraInfo, self.get_camera_info, queue_size=1)
        rospy.Subscriber('pixy2_resolution', PixyResolution, self.get_camera_info, queue_size=1)

        # Wait until we actually have the camera data
        while self.image_width == 0 or self.image_height == 0:
            rospy.sleep(1)
                    
        # Wait for the depth image to become available
        rospy.loginfo("Waiting for range topic...")
        
        rospy.wait_for_message('sonars', Range)
                    
        # Subscribe to the depth image
        self.range_subscriber = rospy.Subscriber("sonars", Range, self.update_range, queue_size=1)

        # Subscribe to the block_data topic and set the callback to update the robot's motion
        # rospy.Subscriber('roi', RegionOfInterest, self.set_cmd_vel, queue_size=1)
        rospy.Subscriber('block_data', PixyData, self.update_block, queue_size=1)
        
        # Wait until we have a block_data to follow
        rospy.loginfo("Waiting for messages on /block_data...")
        rospy.wait_for_message('block_data', PixyData)
        
        rospy.loginfo("block_data messages detected. Starting tracker...")
    
    def load_params(self):
        
        # What (class) should be tracked?
        self.track = rospy.get_param("~track", "person")
        
        # How often should we update the robot's motion?
        self.rate = rospy.get_param("~rate", 10)

        # The max linear speed in meters per second
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.3)
        
        # The minimum linear speed in meters per second
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.02) 

        # The maximum rotation speed in radians per second
        self.max_rotation_speed = rospy.get_param("~max_rotation_speed", 2.0)
        
        # The minimum rotation speed in radians per second
        self.min_rotation_speed = rospy.get_param("~min_rotation_speed", 0.5)
        
        # Sensitivity to target displacements.  Setting this too high
        # can lead to oscillations of the robot.
        self.gain = rospy.get_param("~gain", 2.0)
        
        # The x threshold (% of image width) indicates how far off-center
        # the ROI needs to be in the x-direction before we react
        self.x_threshold = rospy.get_param("~x_threshold", 0.1)

        # How far away from the goal distance (in meters) before the robot reacts
        self.z_threshold = rospy.get_param("~z_threshold", 0.05)
        
        # The maximum distance a target can be from the robot for us to track
        self.max_z = rospy.get_param("~max_z", 1.2)

        # The minimum distance to respond to
        self.min_z = rospy.get_param("~min_z", 0.1)
        
        # The goal distance (in meters) to keep between the robot and the person
        self.goal_z = rospy.get_param("~goal_z", 0.5)

        # How much do we weight the goal distance (z) when making a movement
        self.z_scale = rospy.get_param("~z_scale", 0.5)

        # How much do we weight (left/right) of the person when making a movement        
        self.x_scale = rospy.get_param("~x_scale", 2.0)

        # Slow down factor when stopping
        self.slow_down_factor = rospy.get_param("~slow_down_factor", 0.8)

        # size of ring buffer to calculate average offset
        self.ring_buffer_size = rospy.get_param("~ring_buffer_size", 1)

        # ring buffer to get average of x_offset
        self.ring_buffer_x = collections.deque(maxlen=self.ring_buffer_size)

        # timeout before canceling follow if no target
        self.follow_timeout = rospy.get_param("~follow_timeout", 10)

    def update_block(self, msg):
        #for block in msg.blocks:
        #if len(msg.blocks) > 0 and msg.blocks[0].age > 5 and msg.blocks[0].signature == self.track:
        
        if len(msg.blocks) > 0 and msg.blocks[0].signature == self.track:
            #return blocks[0].index
            block = msg.blocks[0]
            # rospy.loginfo(block.signature)
            if block.roi.height < 5 or block.roi.width < 5 or block.signature <> self.track:
                rospy.loginfo("SKIPPING %s id: %d age: %d" , block.signature, block.index, block.age )
                self.target_visible = False
            else:        
                # If the ROI stops updating this next statement will not happen
                self.target_visible = True
                rospy.loginfo_throttle(1,"TARGET IN BLOCK")

                # set timestamp if target is available
                self.last_target_time = rospy.Time.now()

                # set global ROI from PixyData
                self.roi = block.roi
        else:
            rospy.loginfo_throttle(1,"NO TARGET IN BLOCK")
            self.target_visible = False

    def set_cmd_vel(self):
        
        try:
            if self.target_visible:

                # self.ring_buffer_x.append(self.roi.x_offset)
                # avg_x = sum(self.ring_buffer_x)/self.ring_buffer_size
                # Compute the displacement of the ROI from the center of the image
                # target_offset_x = msg.x_offset + msg.width / 2 - self.image_width / 2
                # target_offset_x = block.roi.x_offset - self.image_width / 2
                # target_offset_x = avg_x - self.image_width / 2
                target_offset_x = self.roi.x_offset - self.image_width / 2
                
                try:
                    percent_offset_x = float(target_offset_x) / (float(self.image_width) / 2.0)
                except:
                    rospy.loginfo("EXCEPTION percent_offset_x")
                    percent_offset_x = 0
                rospy.loginfo("Detected: %d pixel (%d%%) width: %d height: %d range: %f", target_offset_x, percent_offset_x*100, self.roi.height, self.roi.width, self.range)
                # Rotate the robot only if the displacement of the target exceeds the threshold
                if abs(percent_offset_x) > self.x_threshold:
                    # Set the rotation speed proportional to the displacement of the target
                    try:
                        speed = self.gain * percent_offset_x
                        if speed < 0:
                            direction = -1
                        else:
                            direction = 1
                        self.move_cmd = Twist()
                        self.move_cmd.angular.z = -direction * max(self.min_rotation_speed,
                                                    min(self.max_rotation_speed, abs(speed)))
                    except:
                        rospy.loginfo("EXCEPTION speed move")
                        self.move_cmd = Twist()
                else:
                    # Stop rotation
                    self.move_cmd.angular.z = 0

                    # Stop the robot's forward/backward motion by default
                    linear_x = 0
                                                                               
                    # Check the current range against the minimum range
                    if self.range > self.min_z:
                        # Check the max range and goal threshold
                        if self.range < self.max_z and (abs(self.range - self.goal_z) > self.z_threshold):
                            speed = (self.range - self.goal_z) * self.z_scale
                            linear_x = copysign(min(self.max_linear_speed, max(self.min_linear_speed, abs(speed))), speed)
            
                    if linear_x == 0:
                        # Stop the robot smoothly
                        self.move_cmd.linear.x *= self.slow_down_factor
                    else:
                        self.move_cmd.linear.x = linear_x
            else:
                rospy.loginfo_throttle(1,"NO TARGET IN ROI")
                self.target_visible = False

        finally:
            #rospy.loginfo("set_cmd_vel finally")
            return
            

    def update_range(self, msg):
        self.range = msg.range

    def get_camera_info(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)   


    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # append the seeds for the fibonacci sequence
        self._feedback.status = "FEEDBACK " + `goal.signature` 

        
        # publish info to the console for the user
        rospy.loginfo('%s: Executing, FollowAction of signature %i with feedback: %s' % (self._action_name, goal.signature, self._feedback.status))
        
        # # start executing the action
        # for i in range(1, goal.signature):
        #     # check that preempt has not been requested by the client
        #     if self._as.is_preempt_requested():
        #         rospy.loginfo('%s: Preempted' % self._action_name)
        #         self._as.set_preempted()
        #         success = False
        #         break
        #     self._feedback.status = "FOLLOWING goal.signature: " + `i` 
        #     # publish the feedback
        #     self._as.publish_feedback(self._feedback)
        #     # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        #     r.sleep()

        r.sleep()  

        if success:
            self._result.outcome = self._feedback.status
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


    def execute_cb_follow(self, goal):
        
        self.last_target_time = rospy.Time.now()
        r = rospy.Rate(self.rate) 
        
        success = True
        
        # append the seeds for the fibonacci sequence
        self._feedback.status = "FEEDBACK " + str(goal.signature)
        
        # publish info to the console for the user
        rospy.loginfo('%s: Executing, FollowAction of signature %i with feedback: %s' % (self._action_name, goal.signature, self._feedback.status))
        self.track = goal.signature   
        # Begin the tracking loop
        while not rospy.is_shutdown():

            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._result.outcome = "PREEMPTED"
                self._as.set_preempted(self._result, "PREEMPTED")
                success = False
                return

            # Acquire a lock while we're setting the robot speeds
            self.lock.acquire()
            self.set_cmd_vel()
    
            try:
                # If the target is not visible, stop the robot
                if not self.target_visible:
                    # Search if no target
                    duration = rospy.Time.now() - self.last_target_time 

                    if duration.to_sec() > self.follow_timeout:
                        
                        # Rotate to find target
                        self.move_cmd = Twist()
                        # self.move_cmd.angular.z = self.min_rotation_speed
                        self._result.outcome = "CANCEL FOLLOWING: No target for " + str(duration.to_sec()) + "sec..."
                        rospy.loginfo(self._result.outcome)
                        self._as.set_aborted(self._result, "Aborted - CANCEL FOLLOWING")
                        success = False
                        return
                    else:
                        self._feedback.status = "WAITING for target since " + str(duration.to_sec()) + " secs..."
                        rospy.loginfo_throttle(1,self._feedback.status)
                        
                        self.move_cmd = Twist()
        
                else:
                #     # Reset the flag to False by default
                #     if (rospy.Time.now().to_sec() % 5) == 0:
                #         rospy.loginfo("FALSE for ")
                #         self.target_visible = False
                    self._feedback.status = "FOLLOWING " + str(goal.signature)
                    
                # Send the Twist command to the robot
                self.cmd_vel_pub.publish(self.move_cmd)

                # publish the feedback
                self._as.publish_feedback(self._feedback)
                
            finally:
                # Release the lock
                self.lock.release()
                
            # Sleep for 1/self.rate seconds
            r.sleep()
        
        if success:
            self._result.outcome = self._feedback.status
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)



# main for ObjectTracker
# if __name__ == '__main__':
#     try:
#         ObjectTracker()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         rospy.loginfo("Object tracking node terminated.")

if __name__ == '__main__':
    rospy.init_node('object_follow_action')
    rospy.loginfo("object_follow_action node started")
    rospy.loginfo(rospy.get_caller_id() + " object_follow_action node started")
    server = FollowAction(rospy.get_name())

    # test following
    #server.execute_cb_follow()
    
    rospy.spin()

