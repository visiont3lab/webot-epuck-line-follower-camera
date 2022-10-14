"""epuck_go_forward controller."""
from controller import Robot, Motor, DistanceSensor
import numpy as np
import time

# https://cyberbotics.com/doc/guide/tutorial-1-your-first-simulation-in-webots?tab-language=python
# https://cyberbotics.com/doc/guide/tutorial-4-more-about-controllers?tab-language=python
# https://cyberbotics.com/doc/guide/epuck
# https://github.com/KajalGada/Youtube-Tutorial-Download-Material

def position_example():

    # create the Robot instance.
    robot = Robot()
    
    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    
    
    # get the motor devices
    leftMotor = robot.getDevice('left wheel motor')
    rightMotor = robot.getDevice('right wheel motor')
    
    # set the target position of the motors
    # motor run at max speed and then stop when the wheel reach 10rad
    leftMotor.setPosition(10.0)
    rightMotor.setPosition(10.0)
    
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
    
        
        # Read the sensors:
        # Enter here functions to read sensor data, like:
        #  val = ds.getValue()
    
        # Process sensor data here.
    
        # Enter here functions to send actuator commands, like:
        #  motor.setPosition(10.0)
        pass
    


# This duration is specified in milliseconds and it must be a multiple 
# of the value in the basicTimeStep field of the WorldInfo node
#timestep = int(robot.getBasicTimeStep())
TIME_STEP = 32
MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# initialize devices
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)
   
# initialize motor
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

 
# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    psValues = []
    #print("----")
    for i in range(8):
        psValues.append(ps[i].getValue())
        #print(ps[i].getValue())
        
    # detect obstacles
    th = 90.0 # 0 4096
    right_obstacle = psValues[0] > th or psValues[1] > th or psValues[2] > th
    left_obstacle = psValues[5] > th or psValues[6] > th or psValues[7] > th
    
    #print(right_obstacle, left_obstacle)
    
    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed  = 0.8 * MAX_SPEED
    rightSpeed = 0.8 * MAX_SPEED
    
    # modify speeds according to obstacles
    if left_obstacle:
        # turn right
        leftSpeed  = 1 * MAX_SPEED
        rightSpeed = -1 * MAX_SPEED
    elif right_obstacle:
        # turn left
        leftSpeed  = -1 * MAX_SPEED
        rightSpeed = 1 * MAX_SPEED
        
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    pass
      
