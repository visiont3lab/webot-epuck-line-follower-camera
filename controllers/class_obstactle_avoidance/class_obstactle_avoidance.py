"""class_obstactle_avoidance controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import time
import numpy as np
import cv2

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

#ps = []
#psNames = [
#    'ps0', 'ps1', 'ps2', 'ps3',
#    'ps4', 'ps5', 'ps6', 'ps7'
#]

#for i in range(8):
#   ps.append(robot.getDevice(psNames[i]))
#   ps[i].enable(TIME_STEP)

# Ruota a sinistra se appare un oggetto   
ps0 = robot.getDevice('ps0')
ps0.enable(timestep)
ps1 = robot.getDevice('ps1')
ps1.enable(timestep)
ps2 = robot.getDevice('ps2')
ps2.enable(timestep)
ps3 = robot.getDevice('ps3')
ps3.enable(timestep)

# Ruota a destra se appare un oggetto   
ps4 = robot.getDevice('ps4')
ps4.enable(timestep)
ps5 = robot.getDevice('ps5')
ps5.enable(timestep)
ps6 = robot.getDevice('ps6')
ps6.enable(timestep)
ps7 = robot.getDevice('ps7')
ps7.enable(timestep)
   
   
cam = robot.getDevice('camera')
cam.enable(4*timestep)

# Inital velocity
v = 5.0

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
   
    # Enter here functions to read sensor data, like:
    v_ps0 = ps0.getValue()
    v_ps1 = ps1.getValue()
    v_ps2 = ps2.getValue()
    v_ps3 = ps3.getValue()
    v_ps4 = ps4.getValue()
    v_ps5 = ps5.getValue()
    v_ps6 = ps6.getValue()
    v_ps7 = ps7.getValue()
    
    #v_cam = cam.getValue()
    img = cam.getImageArray()
    img = np.asarray(img, dtype=np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    crop_img = cv2.flip(img, 1)
    
    cv2.imshow("Image", crop_img)
    cv2.waitKey(1)
    # Process sensor data here.
  
    vl = v
    vr = v
    if v_ps0>80 or v_ps1>80 or v_ps2>80 or v_ps3>80:
        vl = -v
        
    if v_ps4>80 or v_ps5>80 or v_ps6>80 or v_ps7>80:
        vr = -v


    # Parlo con i motori
    left_motor.setVelocity(vl)
    right_motor.setVelocity(vr)
    
# Enter here exit cleanup code.
