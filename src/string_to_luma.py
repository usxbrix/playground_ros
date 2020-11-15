#!/usr/bin/env python3

"""
    string_to_luma.py - Version 0.1 2020 

    
    Subscribe to text and write to luma display in terminal mode
    

"""

import rospy

from std_msgs.msg import String, Int8
from rosgraph_msgs.msg import Log

# import thread
# import collections

from luma.core.interface.serial import i2c, spi
from luma.core.render import canvas
from luma.oled.device import ssd1306, ssd1309, ssd1325, ssd1331, sh1106

import os
import time
#from demo_opts import get_device
from luma.core.virtual import terminal
from PIL import ImageFont


def make_font(name, size):
    font_path = os.path.abspath(os.path.join(
        os.path.dirname(__file__), 'fonts', name))
    return ImageFont.truetype(font_path, size)


class String2luma(object):


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
    # lock = thread.allocate_lock()
    
    # rev.1 users set port=0
    # substitute spi(device=0, port=0) below if using that interface
    serial = i2c()

    # substitute ssd1331(...) or sh1106(...) below if using that device
    #device = ssd1306()
    device = sh1106()
    
    term = terminal(device)

    def __init__(self, name):
        self.load_params()

        

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        rospy.logdebug("setting up luma display")
        # rev.1 users set port=0
        # substitute spi(device=0, port=0) below if using that interface
        self.serial = i2c(port=1, address=0x3C)

        # substitute ssd1331(...) or sh1106(...) below if using that device
        self.device = sh1106(self.serial, width=128, height=128 )

        rospy.loginfo("luma display ready")
        #for fontname, size in [(None, None), ("tiny.ttf", 6), ("ProggyTiny.ttf", 16), ("creep.bdf", 16), ("miscfs_.ttf", 12), ("FreePixel.ttf", 12), ('ChiKareGo.ttf', 16)]:
        font = make_font("miscfs_.ttf", 12)
        #font = make_font('ChiKareGo.ttf', 16)
        #font = make_font("ProggyTiny.ttf", 16)
        #font = make_font("FreePixel.ttf", 12)
        self.term = terminal(self.device,font,animate=False , word_wrap=True)
        self.term.println("LUMA TERMINAL READY")
        self.term.println("-------------------")
        #self.term.puts("fast??\r\n")
        #self.term.println("--------------")
        #self.term.println("1")
        #self.term.println("A very long new new new new line")


        # Publisher to control the robot's movement
        # self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)



        # Subscribe to topic 
        #rospy.Subscriber(self.topic, String, self.print_string, queue_size=100)
        rospy.Subscriber("/rosout_agg", Log, self.print_string, queue_size=100)
                    
        # Wait for topic to become available
        rospy.logdebug("waiting for topic...")
        #rospy.wait_for_message(self.topic, String)
        rospy.wait_for_message("/rosout_agg", Log)
        rospy.loginfo("Luma terminal ready")
                    

    
    def load_params(self):
        
        # What (class) should be tracked?
        self.topic = rospy.get_param("~topic", "~text")
        
        self.rate = rospy.get_param("~rate", 10)

        self.display_type = rospy.get_param("~type", "sh1106")
        self.display_width = rospy.get_param("~width", 128)
        self.display_height = rospy.get_param("~height", 64)
        self.display_animation = rospy.get_param("~animation", False)



    def print_string(self,msg):
        #self.term.println(str(msg.data))
        self.term.println("{}: {}".format(msg.name, msg.msg))
    


    def shutdown(self):
        rospy.loginfo("stopping...")

        rospy.sleep(1)   


   


# main for 
if __name__ == '__main__':
    rospy.init_node('luma_terminal')
    rospy.loginfo(rospy.get_name() + " node started")
    try:
         server = String2luma(rospy.get_name())
         rospy.spin()
    except rospy.ROSInterruptException:
         rospy.loginfo(rospy.get_name() + "exit...")


