import sys, termios, tty, os, time,rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from random import *



msg = """
Program is used control a rover and a parrot drone using keyboard bindings.
---------------------------
           Rover
---------------------------
   i    ----> Forward
   k    ----> Backward
   j    ----> Pivot Left
   l    ----> Pivot Right
   u    ----> Slow Down
   o    ----> Speed Up
   ;    ----> Motors Disable
   h    ----> Motors Re-enable
   [/]  ----> Control camera tilt angle
   c    ----> Start/Stop recording
All other keys will reset everything to 0

Wait for the program to tell you to start sending commands
CTRL-C to quit
"""

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


rover_motion = rospy.Publisher('rover/cmd_vel',String,queue_size=2)

rospy.init_node('rover_teleop_node',anonymous=True)
time.sleep(2)


while True:    
    ch=getch()
    if ch == '\x03':
        exit(0)
    rover_motion.publish(ch)
