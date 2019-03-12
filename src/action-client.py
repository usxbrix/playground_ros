#! /usr/bin/env python

import rospy


# Brings in the SimpleActionClient
import actionlib

#import Adafruit_BBIO.GPIO as GPIO

# Brings in the messages used by the follow action, including the
# goal message and the result message.
import playground_ros.msg

def feedback_cb(feedback):
    print('[Feedback] Follow status: %s'%(feedback.status))

def follow_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FollowAction) to the constructor.
    client = actionlib.SimpleActionClient('object_follow_action', playground_ros.msg.FollowAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = playground_ros.msg.FollowGoal(signature=2)

    # Sends the goal to the action server.
    client.send_goal(goal, feedback_cb=feedback_cb)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    client.get_result()  # A FollowResult
    print('[Result] State: %d'%(client.get_state()))
    print('[Result] Status Text: %s'%(client.get_goal_status_text()))
    print('[Result] Outcome: %s'%(client.get_result().outcome))
    

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('follow_client_py')
        r = rospy.Rate(1)
        
        # GPIO.setup("GPIO2_4", GPIO.IN)
        # if GPIO.input("P8_10"):
        #     print("HIGH")
        # else:
        #     print("LOW")
        # GPIO.wait_for_edge("GPIO2_4", GPIO.BOTH)
        # print("MODE edge")

        # GPIO.add_event_detect("GPIO2_4", GPIO.RISING) 
        # while not rospy.is_shutdown():
        #     if GPIO.event_detected("MODE"):
        #         print("MODE PUSHED")
            
        #     r.sleep()

        follow_client()
        # result = follow_client()
        # print("Result:", ', ', result.outcome)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")