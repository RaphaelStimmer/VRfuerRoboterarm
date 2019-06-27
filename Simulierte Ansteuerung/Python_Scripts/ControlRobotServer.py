print("Started ControlRobotServer.py for Unity in Python2.")

import socket
import datetime
import time
import os

HOST = '192.168.20.121'  # Standard loopback interface address (localhost)
PORT = 27015        # Port to listen on (non-privileged ports are > 1023)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    
    s.bind((HOST, PORT))
    print("Bind to IP:" + HOST + " and PORT: " + str(PORT))
    s.listen(0)
except OSError as msg:
    print("Exception!!!")
    s.close()
    s = None


if s is None:
    print('Could not open socket!')
    print("Change the host ip! For reading the own ip call ifconfig.")
    sys.exit(1)       

print("Socket online " + str(datetime.datetime.now()))

fileDir = os.path.dirname(os.path.realpath('__file__'))
FilePath = os.path.join(fileDir, "Python_Scripts" ,"Log_Files", str(datetime.datetime.now()) + "_SocketLogFile.txt")
#FilePath = "./Log_Files/" + str(datetime.datetime.now()) + "_SocketLogFile.txt"
LogFile = open(FilePath,"w")
LogFile.write(FilePath + "\n")
LogFile.close()

# Roboter library
import RoboSimClass

# Init and connect to Roboter
target_Robot = RoboSimClass.RoboSimClass()
# Make sure there is only one ground plate in RVIS simulation.
target_Robot.addGroundPlate()
target_Robot.removeGroundPlate()
target_Robot.addGroundPlate()

# Keep server online and ready for new data.
while True:

    conn, addr = s.accept()
    print("File open")
    LogFile = open(FilePath,"a")

    print("Connected by " + str(addr) + " at " +str(datetime.datetime.now()))
    
    # As long as there is data received.
    while not target_Robot.getRospyIsShutDown():

        data = conn.recv(1024)
        
        if not data:
            conn.close()
            LogFile.close()
            print("File close")
            break
        
        else:
            print("### Input data from client.")
            print(data)
            LogFile.write(str(data)+"\n")
            conn.sendall(data)
            # Control Roboter
            target_Robot.act(pose_data=data)
            Robot_Joints = target_Robot.getCurrentJoints()
            print("### Robot_Joints")
            print(Robot_Joints)
            target_Robot.stopIt()
