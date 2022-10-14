"""epuck_go_forward controller."""
from controller import Robot, Motor, DistanceSensor
import numpy as np
import time
import cv2

# https://cyberbotics.com/doc/guide/tutorial-1-your-first-simulation-in-webots?tab-language=python
# https://cyberbotics.com/doc/guide/tutorial-4-more-about-controllers?tab-language=python
# https://cyberbotics.com/doc/guide/epuck
# https://github.com/KajalGada/Youtube-Tutorial-Download-Material
# https://cyberbotics.com/doc/reference/robot?tab-language=python#wb_robot_get_device
# https://cyberbotics.com/doc/reference/display?tab-language=python

    
def run_robot(robot):
    time_step = 32
    max_speed = 6.28* 0.5
    
    # Camera
    camera = robot.getCamera('camera')
    camera.enable(4*time_step)
    #print("Camera width: " , camera.getWidth(), camera.getHeight())
     
    # Motors
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    while robot.step(time_step) != -1:
        
        # Set motor speed
        left_speed = max_speed  
        right_speed = max_speed  
          
        # ------------------------- 
        # ---- Get camera image
        # -------------------------
     
        # --- Option 1
        #cameraData = camera.getImage();
        #image = np.frombuffer(cameraData, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
        #crop_img = image.copy()
        
        # --- Option 2
        img = camera.getImageArray()
        img = np.asarray(img, dtype=np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
        img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        crop_img = cv2.flip(img, 1)
        # -------------------------
        
        # -------------------------
        # ---- Implement Line Controller 
        # -------------------------
        
        gray = cv2.cvtColor(crop_img, cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(gray,(5,5),0)
        ret,thresh = cv2.threshold(blur,127,255,cv2.THRESH_BINARY_INV)
        contours,hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
        
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
    
            cv2.line(crop_img,(cx,0),(cx,720),(255,0,0),1)
            cv2.line(crop_img,(0,cy),(1280,cy),(255,0,0),1)
            cv2.drawContours(crop_img, contours, -1, (0,255,0), 1)
            
            #print(cx,cy)
            
            if (cx >= 108):
                #print ("Turn Right!")
                right_speed = -max_speed*0.25
            elif (cx < 180 and cx > 42):
                #print("On Track!")
                pass
            elif (cx <= 42):
                #print("Turn Left")
                left_speed = -max_speed*0.25
            else:
                print("I don't see the line")
        
        # Visualizaiton
        #cv2.imshow("Image:", crop_img)
        #cv2.waitKey(33)
      
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
    

if __name__ == "__main__":
    robot = Robot()
    run_robot(robot)
   
   