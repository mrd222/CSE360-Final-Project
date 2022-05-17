# This is the code that is my attempt to implement some techniques we learned in class that includes; obstacle avoidance, blob detection/color filtering
# I was ultimately unsuccessful so I decided to take these portions out during the challenge so I can productively participate.  

import socket
import time
from tkinter import W
from turtle import position
import numpy as np
import cv2
from Motor import* 
PWM=Motor()

from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

def turn_left():
    PWM.setMotorModel(-1000,-1000,1000,1000)

def turn_right():
    PWM.setMotorModel(1000,1000,-1000,-1000)

def stop():
    PWM.setMotorModel(0,0,0,0)

def forwards():
    PWM.setMotorModel(1000,1000,1000,1000)

def backwards():
    PWM.setMotorModel(-1000,-1000,-1000,-1000)

cap = cv2.VideoCapture()
params = cv2.SimpleBlobDetector_Params()

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # It converts the BGR color space of image to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # lower mask (0-10)
    lower_yellow = np.array([20,120,120])
    upper_yellow = np.array([29, 255, 255])
    mask0 = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    # join masks
    mask = mask0

    # The black region in the mask has the value of 0,
    # so when multiplied with original image removes all non-blue regions
    result = cv2.bitwise_and(frame, frame, mask = mask)

    # Our operations on the frame come here
    gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 5)
    
    detector = cv2.SimpleBlobDetector_create(params)
    
    biggest = 0
    if detector is not None: 
        for i in detector[0, :]:
            if i[2] > biggest:
                biggest = i[2]
                duck = i
        radius = duck [2]
        x = duck[0]
        y = duck[1]
        
        if x > 420: #width of camera is 640
            print("I SEE A DUCK TO THE RIGHT\n")
            turn_right()
            #time.sleep(1)
        elif x < 220:
            print("I SEE A DUCK TO THE LEFT\n")
            turn_left()
            #time.sleep(1)
        else:
            print("I SEE A DUCK IN FRONT\n")
            if radius > 130:
                backwards()
            elif radius < 100:
                forwards()
            else: 
                stop()

    if detector is None:
        stop()
                
    #cv2.imshow("detected circles", gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        stop()
        break

positions = {}
rotations = {}

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz
        
if __name__ == "__main__":

    IP_ADDRESS = '192.168.0.207'
    clientAddress = "192.168.0.25"
    optitrackServerAddress = "192.168.0.4"
    robot_id = 7

    streaming_client = NatNetClient()
    streaming_client.set_client_address(clientAddress)
    streaming_client.set_server_address(optitrackServerAddress)
    streaming_client.set_use_multicast(True)
    
    streaming_client.rigid_body_listener = receive_rigid_body_frame
    
    is_running = streaming_client.run()


    # Connect to the robot
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((IP_ADDRESS, 5000))
    print('Connected')

    waypoint_count = 0
    kω = 700
    kd = 1700
    r = .8
    p_d = [5.4, 0.]
    p_b = np.array([0,0])
    kb = 1
    q2 = np.array[(0,0)] # for obstacle avoidance

    try:
        while is_running:
            if robot_id in positions:
                if(waypoint_count == 0):
                    p_d = [5.4, 0]
                elif(waypoint_count == 1):
                    p_d = [5.4, 2.]
                elif(waypoint_count == 2):
                    p_d = [5.3, 3.5]
                elif(waypoint_count == 3):
                    p_d = [2., 3.5]
                elif(waypoint_count == 4):
                    p_d = [1.,3.5]
                elif(waypoint_count == 5):
                    p_d = [1 ,2]
                elif(waypoint_count == 6):
                    p_d = [0., 0.]
                elif(waypoint_count == 7):
                    p_d = [2.,0.]
                elif(waypoint_count == 8):
                    p_d = [5.4, 0.]
                elif(waypoint_count == 9):
                    p_d = [5.4, -1.]
                elif(waypoint_count == 10):
                    p_d = [5.4, -2.9]
                elif(waypoint_count == 11):
                    p_d = [2., -2.9]
                elif(waypoint_count == 12):
                    p_d = [0, -2.9]
                elif(waypoint_count == 13):
                    p_d = [0., 1.]
                elif(waypoint_count == 14):
                    p_d = [0.,0.]
                elif(waypoint_count == 15):
                    p_d = [2., 0.]
                elif(waypoint_count == 16):
                    p_d = [5.4, 0.]
                elif(waypoint_count == 17):
                    p_d = [0., 0.]
                elif(waypoint_count == 18):
                    p_d = [1.,2.]
                elif(waypoint_count == 19):
                    p_d = [2, 3.]
                elif(waypoint_count == 20):
                    p_d = [-4.,3.8]
                elif(waypoint_count == 21):
                    p_d = [-4., 2]
                elif(waypoint_count == 22):
                    p_d = [-4.,0.]
                elif(waypoint_count == 23):
                    p_d = [0.,0.]
                elif(waypoint_count == 23):
                    p_d = [5.4, 0.]
                elif(waypoint_count == 24):
                    p_d = [0.,0.]
                elif(waypoint_count == 25):
                    p_d = [-4., 0]
                elif(waypoint_count == 26):
                    p_d = [-4., -1]
                elif(waypoint_count == 27):
                    p_d = [-4., -3]
                elif(waypoint_count == 28):
                    p_d = [-2., -3.]
                elif(waypoint_count == 29):
                    p_d = [0.,-2.9]
                elif(waypoint_count == 30):
                    p_d = [0.,0.]
                elif(waypoint_count == 31):
                    p_d = [5.4, 0]
                
                p_r = [positions[robot_id][0], positions[robot_id][1]]
                
                α = np.arctan2((p_d[1]-p_r[1]), (p_d[0]-p_r[0]))
                q = (p_d[0] - p_r[0] , p_d[1] - p_r[1])
                theta_d = np.degrees(np.arctan2(q2[1],q2[0]))
                distance = np.sqrt(q[0]**2+q[1]**2)
                theta_r = np.radians(rotations[robot_id])

                ω = np.degrees(kω*np.arctan2(np.sin(α-theta_r),np.cos(α-theta_r)))
                v = kd*(distance)
                
                u = np.array([v - ω, v + ω])
                u[u > 1500] = 1500
                u[u < -1500] = -1500
                

                if(distance <= r):
                    waypoint_count += 1
                    #command = 'CMD_MOTOR#0000#0000#-1500#-1500\n'
                    #s.send(command.encode('utf-8'))
                    time.sleep(1)

                # Send control input to the motors
                command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
                s.send(command.encode('utf-8'))
                print("Robot: ", p_r,  "\nDestination: ", p_d, "\nDistance: ", distance)
                print("ω: ", ω, "\tv: ", v, "\tu: ", u, "\np_r: ", p_r, "\ttheta_r: ", theta_r, "\ttheta_d: ", α, "\ndistance: ", distance)

                # Wait for 1 second
                time.sleep(.1)


    except KeyboardInterrupt:
        # STOP
        command = 'CMD_MOTOR#00#00#00#00\n'
        s.send(command.encode('utf-8'))
        s.shutdown(2)
        s.close()

    # Close the connection
