# Das Programm liest die Dummydaten(Zielpose des Roboters) von einem Textfile
# und schickt diese an Server weiter.
# Client verhaelt sich wie Unity -> Unity-Simulator
#Python Socket
print("Socket Client with Unity Python 2")

# Echo client program

import socket
import time

Filename = raw_input("Enter data filename: ") 

HOST = '192.168.20.105'    # The remote host
PORT = 27015              # The same port as used by the server

SocketServer = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    SocketServer.connect((HOST, PORT))

except OSError as msg:
    print("Exception!!!")
    SocketServer.close()
    SocketServer = None

if SocketServer is None:
    print('Could not open socket!')
    print("Change the host ip! For reading the own ip call ifconfig.")
    sys.exit(1)   

def stringTofloat(inputString):
    s = [s for s in inputString if ord(s) < 58 and ord(s) > 47 or s == '.'
        or s == '-' or s == 'E' or s == 'e']
    x = ''.join([ y for y in s])
    return float(x)

def decodeLogFileLine(LineData):

    return 0


LogFile = open(Filename,"r")
#Read first line because its the file name
print(LogFile.readline())

for x in range(10):
    newLine = LogFile.readline()
    if newLine == "":
        print("Stop reading file at x = ")
        print(x)
        # End of file or blank line -> stop reading the file
        break
    else:
        ElementList = newLine.split(',')
        
        FloatPosition=0
        xPosition = stringTofloat(ElementList[0])
        yPosition = stringTofloat(ElementList[1])
        zPosition = stringTofloat(ElementList[2])

        xRotation = stringTofloat(ElementList[3])
        yRotation = stringTofloat(ElementList[4])
        zRotation = stringTofloat(ElementList[5])

        wRotation = stringTofloat(ElementList[6])

        SendString = '{},{},{},{},{},{},{}'.format(xPosition, yPosition, zPosition, xRotation, yRotation, zRotation, wRotation)

        print(SendString)
        my_bytes = bytearray(SendString)
        #my_bytes = bytearray(str(x) + " ", 'utf8')
        SocketServer.sendall(my_bytes)
        time.sleep(15)
        data = SocketServer.recv(1024)
        print('REC',repr(data))

SocketServer.close()

print('Received', repr(data))

#f = open("file.py","rb")
#print(f)




"""
def write(self):
    if not self._request_queued:
        self.queue_request()

    self._write()

    if self._request_queued:
        if not self._send_buffer:
            # Set selector to listen for read events, we're done writing.
            self._set_selector_events_mask('r')

"""