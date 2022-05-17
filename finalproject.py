# Run this

import socket
import time
from tkinter import W
from turtle import position
import numpy as np

from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

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
