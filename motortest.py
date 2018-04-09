#!/usr/bin/python
import sys, termios, tty, os, time
from random import *
from std_msgs.msg import String
import rospy

#Keagan's imports
#from __future__ import print_function
import time
from dual_g2_hpmd_rpi import motors, MAX_SPEED

#Keagan's hideous global variables
global SPEED_INTERVAL
SPEED_INTERVAL = 5
global rover_enabled
rover_enabled = False
global first_execute
first_execute = True
global prev_ch
prev_ch = 'placeholder'
global roverspeed
roverspeed = MAX_SPEED/2 #Keagan  setting initial speed to half of max

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

#Get commands from Keyboard
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

#Keagan -- Check if the key is being held
def key_held(ch,prev_ch):
    if ch == prev_ch:
        return True
    else:
        return False

#Keagan's Rover function
def key_rover(ch,prev_ch,motorspeed,motors_enabled):
    #check if this is the first time executing this function
    while ch:
        print ch
        #Rover Motors Re-Enable
        if ch == 'h':
            if motors_enabled == False:
                motors.enable()
                motors_enabled = True
                print ('Rover Motors Enabled. To disable, press \';\'')

        #Don't allow any other input until motors re-enabled
        elif motors_enabled == False:
            print ('Rover Motors Disabled. To enable, press \'h\'')

        #Rover Motors Disable
        elif ch == ';':
            if motors_enabled == True:
                motors.disable()
                motors_enabled = False
                print ('Rover Motors Disabled. To enable, press \'h\'')
        #Rover Forward
        elif ch == 'i':
            print ('Rover Forward')
            motors.setSpeeds(motorspeed,motorspeed)

        #Rover Backward
        elif ch == 'k':
            print ('Rover Backward')
            motors.setSpeeds(-motorspeed,-motorspeed)

        #Rover Pivot Right
        elif ch == 'l':
            print ('Rover Pivot Right')
            motors.setSpeeds(motorspeed,-motorspeed)

        #Rover Pivot Left
        elif ch == 'j':
            print ('Rover Pivot Left')
            motors.setSpeeds(-motorspeed,motorspeed)

        #Rover Speed Up
        elif ch == 'o':
            #check that the motorspeed is not MAX_SPEED
            if(motorspeed == MAX_SPEED):
                print ('Rover is at maximum defined speed of %d.', motorspeed)
            else:
                motorspeed = motorspeed + SPEED_INTERVAL;
                #check if the increase put motorspeed over MAX_SPEED & correct
                if(motorspeed >= MAX_SPEED):
                    motorspeed == MAX_SPEED
                print ('Rover Speed Up: %d', motorspeed)

        #Rover Slow Down
        elif ch == 'u':
            #check that the motorspeed is not 0
            if(motorspeed == 0):
                print ('Rover is at minimum speed of %d.', motorspeed)
            else:
                motorspeed = motorspeed - SPEED_INTERVAL;
                #check if the decrease put motorspeed under 0 & correct
                if(motorspeed <= 0):
                    motorspeed == 0
                print ('Rover Slow Down: %d', motorspeed)

        #Rover stop if no key input
        else:
            motors.setSpeeds(0, 0)
            print ('Rover Stop')
            prev_ch = 'Stopped'
        return motorspeed, motors_enabled

#Keagan's PS3 Contoller to Rover Function
#ps3.get_axis(0)
#Left stick left/right: 0
#Left stick up/down: 1     ##this should be the throttle
#Right stick left/right: 2 ##this should be the steering
#Right stick up/down: 3
# For now, I am assuming that all axes of each joystick are passed in,
# even though only the left stick's up/down and the right stick's left/right
# are going to be used.
# Note: deadzone is the area around the origin where it is designed so that 
#       there is no input when the joystick is very close to the origin, but
#       not quite there.  This will be a constant defined later.
def ps3_rover(left_x_axis,left_y_axis,right_x_axis,right_y_axis,motorspeed,motors_enabled,deadzone):
    left_x_axis = ps3_to_motoraxis_conv(left_x_axis,deadzone)
    left_y_axis = ps3_to_motoraxis_conv(left_y_axis,deadzone)
    right_x_axis = ps3_to_motoraxis_conv(right_x_axis,deadzone)
    right_y_axis = ps3_to_motoraxis_conv(right_y_axis,deadzone)
    
    
    
    return motorspeed, motors_enabled

#This function translates the PS3's -1 to DEADZONE U DEADZONE to +1 range into the rover's -480 to +480 range
def ps3_to_motoraxis_conv(ps3,deadzone)
    #When ps3 joystick is positive and greater than deadzone
    if ps3 > deadzone:
        motoraxis = (ps3 - deadzone)*480/(1 - deadzone)
    #When ps3 joystick is negative and less than deadzone
    elif ps3 < -deadzone:
        motoraxis = (ps3 + deadzone)*480/(1 - deadzone)
    #When ps3 joystick is within deadzone
    else:
        motoraxis = 0
    
    #Round off the float to an integer so the motor controller can read it.
    motoraxis = int(round(motoraxis))
    return motoraxis


def callback(data):
     rospy.loginfo("%s", data.data)

     return data.data
##############################################################################
################ End of Definitions, beginning code execution ################
##############################################################################

motors.enable() #Keagan  initializing motors
rover_enabled = True
motors.setSpeeds(0, 0) #Keagan    guaranteeing motors are stopped

print msg
rospy.init_node('Rover',anonymous=True)

#Keagan -- is this where I should place my operating code?
print ('Send commands to rover now...')
while True:
    ch = rospy.Subscriber("rover/cmd_vel",String,callback)
    print ch

    #Exit
    if ch == '\x03':
        motors.disable()
        rover_enabled = False
        exit(0)
    #Re-enabling motors
    if ch == 'h':
        if rover_enabled == False:
            rover_enabled = True
            motors.enable()
            motors.setSpeeds(0, 0)
            print ('Motors Re-enabled. To disable, press \';\'')

    #Operate Rover
    if key_held(ch,prev_ch) == False:
        if rover_enabled == True:
            roverspeed, rover_enabled = key_rover(ch,prev_ch,roverspeed,rover_enabled)

    #Reset the previous character
    prev_ch = ch
