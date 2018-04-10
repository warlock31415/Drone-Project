#Control Drone with keyboard keys
#Extra points if you write a teleop program a joystick


#!/usr/bin/python
import sys, termios, tty, os, time, rospy
from std_msgs.msg import Empty,UInt8,Bool
from geometry_msgs.msg import Twist
from random import *

#Keagan's imports
#from __future__ import print_function
import time
from dual_g2_hpmd_rpi import motors, MAX_SPEED

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
   p    ----> Motors Re-enable
   [/]  ----> Control camera tilt angle
   c    ----> Start/Stop recording
---------------------------
       Parrot Drone
---------------------------
   w/s  ----> Pitch
   a/d  ----> Roll
   q/e  ----> Yaw
   </>  ----> Altitude
   [/]  ----> Control camera tilt angle
   c    ----> Start/Stop recording
   f    ----> (Make sure there are no immediate obstructions)
---------------------------
All other keys will reset everything to 0

Wait for the program to tell you to start sending commands
CTRL-C to quit
"""

#Get commands from Keyboard
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

#Writes values in the form of Twist() message
def move(motion,x_vel,y_vel,z_vel,yaw):
    motion.linear.x = x_vel
    motion.linear.y = y_vel
    motion.linear.z = z_vel
    motion.angular.z = yaw
    return motion

#reset all velocities
def reset():
    lin_x = 0
    lin_y = 0
    lin_z = 0
    ang_z = 0
    return (lin_x,lin_y,lin_z,ang_z,ch)

#move camera up and down
def camera_control(ch,camera,virtual_camera):
    if ch == '[':
        camera.angular.y = camera.angular.y - 0.2
    elif ch == ']':
        camera.angular.y = camera.angular.y + 0.2
    virtual_camera.publish(camera)
    return camera

#map keys to different actions
def key_map(ch,lin_x,lin_y,lin_z,ang_z,camera,virtual_camera):
    w_pressed=0  #Keagan--This isnt used anywhere?
    a_pressed=0  #Keagan--This isnt used anywhere?
    while ch:
        print ch
        #Pitch
        if ch ==  "w":
            if lin_x<0:
                lin_x = 0
            lin_x = lin_x + 0.2  #Keagan--should 0.2 be a 'constant' variable?
            lin_y,lin_z,ang_z = (0,0,0)  # yes I know there are no true constants in Python
        elif ch == 's':
            if lin_x > 0:
                lin_x = 0
            lin_x = lin_x - 0.2
            lin_y,lin_z,ang_z = (0,0,0)
        #Roll
        elif ch == "a":
            if lin_y<0:
                lin_y = 0
            lin_y = lin_y + 0.2
            lin_x,lin_z,ang_z = (0,0,0)
        elif ch == "d":
            if lin_y>0:
                lin_y = 0
            lin_y = lin_y - 0.2
            lin_x,lin_z,ang_z = (0,0,0)
        #yaw
        elif ch == "e":
            if ang_z < 0:
                ang_z = 0
            ang_z = ang_z + 0.2
            lin_y,lin_z,lin_x = (0,0,0)
        elif ch == "q":
            if ang_z>0:
                ang_z = 0
            ang_z = ang_z - 0.2
            lin_y,lin_z,lin_x = (0,0,0)
        #Altitude
        elif ch == ">":
            lin_z = 1
            lin_y,lin_x,ang_z = (0,0,0)
        elif ch =="<":
            lin_z = -1
            lin_y,lin_x,ang_z = (0,0,0)
        #Flip
        elif ch == 'f':
            flip = randint(0,3)
            rospy.loginfo('Flipping')
            flip_off.publish(flip)
            time.sleep(2)
        #Move camera
        elif (ch == '[' or ch == ']'):
            camera = camera_control(ch,camera,virtual_camera)
        #Reset drone's everything if anything else is pressed
        else:
            lin_x,lin_y,lin_z,ang_z,ch = reset()
        return (lin_x,lin_y,lin_z,ang_z,ch)

#Keagan -- Check if the key is being held
def key_held(ch,prev_ch):
    if ch == prev_ch:
        return True
    else:
        return False
    
#Keagan's Rover function
def key_rover(ch,prev_ch):
    #check if this is the first time executing this function
    if first_execute == True:
        prev_ch = 'placeholder'
        first_execute = False
    while ch:
        print ch
        
        #Rover Forward
        if ch == 'i':  
            rospy.loginfo('Rover Forward')
            motors.setSpeed(motorspeed, motorspeed)
            
        #Rover Backward
        elif ch == 'k':
            rospy.loginfo('Rover Backward')
            motors.setSpeed(-motorspeed, -motorspeed)
            
        #Rover Pivot Right
        elif ch == 'l':
            rospy.loginfo('Rover Pivot Right')
            motors.setSpeed(motorspeed, -motorspeed)
            
        #Rover Pivot Left
        elif ch == 'j':
            rospy.loginfo('Rover Pivot Left')
            motors.setSpeed(-motorspeed, motorspeed)
            
        #Rover Speed Up
        elif ch == 'o':
            #check that the motorspeed is not MAX_SPEED
            if(motorspeed == MAX_SPEED):
                rospy.loginfo('Rover is at maximum defined speed of %d.', motorspeed)
            else:
                motorspeed = motorspeed + SPEED_INTERVAL;
                #check if the increase put motorspeed over MAX_SPEED & correct
                if(motorspeed >= MAX_SPEED):
                    motorspeed == MAX_SPEED
                rospy.loginfo('Rover Speed Up: %d', motorspeed)
                
        #Rover Slow Down
        elif ch == 'u':
            #check that the motorspeed is not 0
            if(motorspeed == 0):
                rospy.loginfo('Rover is at minimum speed of %d.', motorspeed)
            else:
                motorspeed = motorspeed - SPEED_INTERVAL;
                #check if the decrease put motorspeed under 0 & correct
                if(motorspeed <= 0):
                    motorspeed == 0
                rospy.loginfo('Rover Slow Down: %d', motorspeed)
                
        #Rover Motors Disable
        elif ch == ';':
            if motors_enabled == True:
                motors.disable()
                motors_enabled = False
                rospy.loginfo('Rover Motors Disabled. To enable, press \'p\'')    
        #Rover stop if no key input
        else:
            motors.setSpeed(0, 0)
            rospy.loginfo('Rover Stop')
            prev_ch = 'Stopped'
        return None

##############################################################################
################ End of Definitions, beginning code execution ################
##############################################################################

#Keagan's hideous global variables
SPEED_INTERVAL = 5
motors_enabled = False
first_execute = True
prev_ch = 'placeholder'
    
flag,ch= (0,0)
lin_x,lin_y,lin_z,ang_z,ch = reset()
motors.enable() #Keagan  initializing motors
motors_enabled = True
motors.setSpeeds(0, 0) #Keagan    guaranteeing motors are stopped
motorspeed = MAX_SPEED/2 #Keagan  setting initial speed to half of max
print msg

#Set up messages and Publishers
motion = Twist()
camera = Twist()
take_off = rospy.Publisher('bebop/takeoff',Empty,queue_size=2)
land = rospy.Publisher('bebop/land',Empty,queue_size=2)
record = rospy.Publisher('bebop/record',Bool,queue_size=2)
motion_command = rospy.Publisher('bebop/cmd_vel',Twist,queue_size=1)
flip_off = rospy.Publisher('bebop/flip',UInt8,queue_size=1)
virtual_camera = rospy.Publisher('bebop/camera_control',Twist,queue_size=2)

#initialize ROS node
rospy.init_node('teleop_node',anonymous=True)
#Wait for initialization to finish
time.sleep(5)

#Keagan -- is this where I should place my operating code?
rospy.loginfo('Send commands to rover now...')
while True:
    ch = getch()
    
    #Exit
    if ch == '\x03':
        motors.disable()
        motors_enabled = False
        exit(0) #Keagan--should this become break so that some
                # code can still execute?
        
    #Re-enabling motors
    if ch == 'p':
        if motors_enabled == False:
            motors_enabled = True
            motors.enable()
            motors.setSpeeds(0, 0)
            rospy.loginfo('Motors Re-enabled. To disable, press \';\'')

    #Operate Rover
    if key_held(ch,prev_ch) == False:
        if motors_enabled == True:
            key_rover(ch,prev_ch)

    #Recording
    if ch == "c":
        if flag == 0:
            rospy.loginfo('Turning on recording...')
            flag = 1
        elif flag == 1:
            rospy.loginfo('Turning off recording...')
            flag = 0
        record.publish(flag)

    #Reset the previous character
    prev_ch = ch
    
#Keagan -- should you have a prompt here so that the drone doesn't
#          automatically take off?
rospy.loginfo('Taking off.....')
take_off.publish(Empty())
time.sleep(5)

#Tell user to start sending commands
rospy.loginfo('Send Commands now..')
while True:   
    ch = getch()
    if ch == '\x03':
        motors.disable() ## Keagan's failsafe
        motors_enabled = False
        exit(0) #Keagan--should this become break so that some
                # code can still execute?
                
    #Map pressed keys
    lin_x,lin_y,lin_z,ang_z,ch=key_map(ch,lin_x,lin_y,lin_z,ang_z,camera,virtual_camera)
    move(motion,lin_x,lin_y,lin_z,ang_z)
    motion_command.publish(motion)
    ##lin_x,lin_y,lin_z,ang_z,ch=reset()
    #Start/Stop recording
    if ch == "c":

        if flag == 0:
            rospy.loginfo('Turning on recording...')
            flag = 1
        elif flag == 1:
            rospy.loginfo('Turning off recording...')
            flag = 0
        record.publish(flag)

#land on exit
land.publish(Empty())
time.sleep(2)

##Keagan--If I am interpreting this correctly, the code somewhat follows this path:
##          Rover moves with user, Rover disabled, Drone moves with user,
##          Drone disabled.  Did we want to edit the code so that it loops back
##          to operating the rover again?


#Keagan -- should we 
