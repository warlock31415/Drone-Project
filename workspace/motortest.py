#Control Drone with keyboard keys
#Extra points if you write a teleop program a joystick


#!/usr/bin/python
import sys, termios, tty, os, time
from random import *

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
        tty.setraw(fd)
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
            motors.setSpeeds(motorspeed, motorspeed)
            
        #Rover Backward
        elif ch == 'k':
            print ('Rover Backward')
            motors.setSpeeds(-motorspeed, -motorspeed)
            
        #Rover Pivot Right
        elif ch == 'l':
            print ('Rover Pivot Right')
            motors.setSpeeds(motorspeed, -motorspeed)
            
        #Rover Pivot Left
        elif ch == 'j':
            print ('Rover Pivot Left')
            motors.setSpeeds(-motorspeed, motorspeed)
            
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

##############################################################################
################ End of Definitions, beginning code execution ################
##############################################################################


    
flag,ch= (0,0)
lin_x,lin_y,lin_z,ang_z,ch = reset()
motors.enable() #Keagan  initializing motors
rover_enabled = True
motors.setSpeeds(0, 0) #Keagan    guaranteeing motors are stopped

print msg

#initialize ROS node
#Wait for initialization to finish

#Keagan -- is this where I should place my operating code?
print ('Send commands to rover now...')
while True:
    ch = getch()
    print ch
    
    #Exit
    if ch == '\x03':
        motors.disable()
        rover_enabled = False
        exit(0) #Keagan--should this become break so that some
                # code can still execute?
        
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

    #Recording
    if ch == "c":
        if flag == 0:
            print ('Turning on recording...')
            flag = 1
        elif flag == 1:
            print ('Turning off recording...')
            flag = 0

    #Reset the previous character
    prev_ch = ch
    
#Keagan -- should you have a prompt here so that the drone doesn't
#          automatically take off?
print ('Taking off.....')

#Tell user to start sending commands
print ('Send Commands now..')
while True:   
    ch = getch()
    if ch == '\x03':
        motors.disable() ## Keagan's failsafe
        rover_enabled = False
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
            print ('Turning on recording...')
            flag = 1
        elif flag == 1:
            print ('Turning off recording...')
            flag = 0
        record.publish(flag)

#land on exit


##Keagan--If I am interpreting this correctly, the code somewhat follows this path:
##          Rover moves with user, Rover disabled, Drone moves with user,
##          Drone disabled.  Did we want to edit the code so that it loops back
##          to operating the rover again?


#Keagan -- should we 
