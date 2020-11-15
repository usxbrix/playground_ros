#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import PoseStamped

from std_msgs.msg import String

from tf.transformations import quaternion_from_euler

from math import radians


class Text2Action:
    range = 0
    speed = 0
    cmd_queue = []
    text_subscriber = None
    mbs_goal_publisher = None
    pose_msg = None
    rate = None

    def __init__(self):

        # init_arguments(self)

        self.rate = rospy.Rate(1)  # 10hz
        # move_base_simple goal publischer
        self.mbs_goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        
        self.text_subscriber = rospy.Subscriber("/dialogflow_text", String, self.process_text)

        self.pose_msg = PoseStamped()
        self.pose_msg.header.seq = 0
        self.pose_msg.header.stamp.secs = 0
        self.pose_msg.header.stamp.nsecs = 0
        self.pose_msg.header.frame_id = 'base_link'
        self.pose_msg.pose.position.x = 0
        self.pose_msg.pose.position.y = 0
        self.pose_msg.pose.position.z = 0
        self.pose_msg.pose.orientation.x = 0
        self.pose_msg.pose.orientation.y = 0
        self.pose_msg.pose.orientation.z = 0
        self.pose_msg.pose.orientation.w = 0


        rospy.loginfo(rospy.get_caller_id() + " init done...")
        while not rospy.is_shutdown():
            if len(self.cmd_queue) > 0:
                next = self.cmd_queue.pop(0)
                rospy.loginfo(rospy.get_caller_id() + " Doing next command: %s", next )
                self.move(next)

            self.rate.sleep()

            

    def process_text(self, text):
        # rospy.loginfo_throttle(1,rospy.get_caller_id() + "Range: %s", range.range)
        rospy.loginfo(rospy.get_caller_id() + " Text: %s", text.data)

        if "stop" in text.data or "Stopp" in text.data:
            rospy.loginfo(rospy.get_caller_id() + " STOPPING!")
            self.cmd_queue = []
            print(self.cmd_queue)
        else:
            self.cmd_queue += str.split(text.data)
            print(self.cmd_queue)

        #            self.mbs_goal_publisher.publish(self.pose_msg)


    def stop_and_turn(self):
        self.pose_msg.linear.x = 0
        rospy.loginfo(rospy.get_caller_id() + " OBSTACLE: stopping")
        self.mbs_goal_publisher.publish(self.pose_msg)
        rospy.sleep(1)
        self.pose_msg.angular.z = self.rotspeed * (-1)**random.randrange(2)
        rospy.loginfo(rospy.get_caller_id() + " OBSTACLE: turning speed %s", self.pose_msg.angular.z)
        self.mbs_goal_publisher.publish(self.pose_msg)
        rospy.sleep(1)
        self.pose_msg.angular.z = 0

    def move(self,cmd):
        if cmd == "links":
            self.pose_msg.pose.position.x = 0.3
            
            self.mbs_goal_publisher.publish(self.pose_msg)

            self.pose_msg = PoseStamped()

        elif cmd == "rechts":
            q = quaternion_from_euler(0, 0, radians(-90))            
            print "The quaternion representation is %s %s %s %s." % (q[0], q[1], q[2], q[3])
            self.pose_msg.pose.orientation.x = q[0]
            self.pose_msg.pose.orientation.y = q[1]
            self.pose_msg.pose.orientation.z = q[2]
            self.pose_msg.pose.orientation.w = q[3]

            self.mbs_goal_publisher.publish(self.pose_msg)

            self.pose_msg = PoseStamped()
        else:
            rospy.loginfo(rospy.get_caller_id() + " NOT A VALID COMMAND: %s", cmd)

        

   


if __name__ == '__main__':

    try:
        # Starts a new node
        rospy.init_node('text2action', anonymous=True)
        t2a = Text2Action()

        # wait half a sec bevor moving
        rospy.sleep(0.5)

        #robot_movement.move(speed)
    except rospy.ROSInterruptException: pass
