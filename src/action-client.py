#! /usr/bin/env python

import rospy


# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the follow action, including the
# goal message and the result message.
import playground_ros.msg

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
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FollowResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('follow_client_py')
        result = follow_client()
        print("Result:", ', ', result.outcome)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")