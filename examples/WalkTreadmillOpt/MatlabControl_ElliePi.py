#
# Copyright (C) Ghost Robotics - All Rights Reserved
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential
# Written by Tom Jacobs <tom.jacobs@ghostrobotics.io>, Avik De <avik@ghostrobotics.io>
#
# Command: Sends BehaviorCmds to the robot to command its behavior.

import struct
import serial
import math
from time import sleep
import ctypes as ct
import socket
import array
import sys
import signal

#USER INPUTS 
tcpPort = 50000
nVariables = 9 #Assumes all variables are of type double
debug = False

#SERIAL STUFF
# Open USB port
portAddress = "/dev/tty.usbserial-DN02MSZX"
#portAddress = "/dev/tty.usbserial-DN01QAKV"
#portAddress = "COM29"
baud = 115200
# baud = 230400
# No timeout so we can keep going with whatever bytes are in waiting
port = serial.Serial(portAddress, baud, timeout=None)
print("Sending to: " + port.name)

#TCP STUFF
#Get number of bytes (all variables should be doubles)
nBytes = nVariables * 8
#Init last commands to zero
mDataLast = [0.0]*nVariables
#Setup TCP connection
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('', tcpPort)) #for anyone to connect to
s.listen(1)
print ("waiting for response from client at port ",PORT)
#function hangs until connected to by Matlab
#TODO: add explicit timeout?
conn, addr = s.accept()
print ('Connected by', addr)

# Receive buffer
rxBuf = ''
#Init main while loop variable to True
run = True

def calculateChecksum(bytes, length):
    checksum = ct.c_ushort(0)
    for i in range(length):
        checksum = ct.c_ushort(int(checksum.value) + ord(bytes[i]));
    return checksum.value

# Handle for getting Ctrl+C input, exits main while loop
def exitHandle(signal, frame):
    global run
    print('\nClosing connection and exiting function')
    run = False

print('Press Ctrl+C to exit function')

while(run):

    #Check if user wants to end function
    signal.signal(signal.SIGINT, exitHandle)

    #Read data from TCP connection
    data = conn.recv(nBytes)
    if len(data) in nBytes:
        mData = array.array('d', data)
        #Check native byte order
        if sys.byteorder in 'little':
            # use big endian for reading from Matlab
            mData.byteswap()
    else:
        mData = mDataLast

    # Old Code for setting up behavior struct ----
    # Init
    # linear = {}
    # angular = {}
    # position = {}
    # orientation = {}

    # Set values
    # id = 0
    # mode = 1 # BehaviorMode_RUN

    # linear['x'] = mData[0]
    # linear['z'] = 0
    # angular['x'] = 0
    # angular['y'] = 0
    # angular['z'] = mData[1]
    # position['x'] = 0
    # position['y'] = 0
    # position['z'] = 0
    # orientation['x'] = 0
    # orientation['y'] = 0
    # orientation['z'] = 0
    # orientation['w'] = 0
    #-------------------------------------------

    #Assign tcp data to variables (for readability)
    fwdVel = mData[0]
    yaw = mData[1]

    var1 = mData[2]
    var2 = mData[3]
    var3 = mData[4]
    var4 = mData[5]
    var5 = mData[6]
    var6 = mData[7]
    var7 = mData[8]
    
    #print("Sending walking speed:  " + str(linear['x']))

    # Pack BehaviorCmd. Definition is at the end of this file in comments.
    # Packet version number, BehaviorCmd: {id, twist.linear, twist.angular, pose.position, pose.orientation, mode}
    
    # Old Code for setting up behavior struct -------------------------------------------------------
    # behavior_command = struct.pack( '<II3f3f3f4fI', 1, # Version 1 of serial packet format
    #                                                 id, 
    #                                                 linear['x'], linear['y'], linear['z'], 
    #                                                 angular['x'], angular['y'], angular['z'],
    #                                                 position['x'], position['y'], position['z'],
    #                                                 orientation['x'], orientation['y'], orientation['z'], orientation['w'],
    #                                                 mode)
    # --------------------------------------------------------------------------------------------------

    # Custom packet for hardware in the loop optimizations
    behavior_command = struct.pack( '<I2f4f3f', 1, # Version 1 of serial packet format
                                                    fwdVel,yaw,
                                                    var1,var2,var3,var4,
                                                    var5,var6,var7)


    # Calculate and append checksum, prepend align bytes
    checksum = calculateChecksum(behavior_command, len(behavior_command))
    behavior_command = struct.pack('<cc', b"G", b"R") + behavior_command + struct.pack('<H', checksum)

    # Send BehaviorCmd via USB
    port.write(behavior_command)

    # Debug Loop: Read incoming data
    if debug:
        rxBuf += port.read(50)
        chunks = rxBuf.split('GR')
        for chunk in chunks:
            if len(chunk) > 0:
                try:
                    # Parse packet into tuple: time, lastRX time, 3 float robot state, checksum 
                    tup = struct.unpack('<2I3fH', chunk)
                except:
                    continue

                # Get times
                time = tup[0]
                lastRXtime = tup[1]

                # Compare checksum
                checksum = tup[5]
                checksumCalculated = calculateChecksum('GR' + chunk, 2+4+4+(3*4))

                # Print state
                if(checksum == checksumCalculated):
                    print("Times: " + str(time) + " " + str(lastRXtime) + " " + str(time-lastRXtime) + " Robot IMU state: " + str(tup[2]) + " " + str(tup[3]) + " " + str(tup[4]))

        # Put back the last (possibly unfinished) chunk
        rxBuf = chunks[-1]
    # Save last commands from Matlab
    mDataLast = mData

# Close Conections
port.close()        
conn.close()
print('Connection closed')


# BehaviorCmd definition:

# typedef struct _BehaviorCmd {
#     uint32_t id;
#     Twist twist;
#     Pose pose;
#     uint32_t mode;
# } BehaviorCmd;

# typedef struct _Twist {
#     Vector3 linear;
#     Vector3 angular;
# } Twist;

# typedef struct _Pose {
#     Vector3 position;
#     Quaternion orientation;
# } Pose;

# typedef struct _Vector3 {
#     float x;
#     float y;
#     float z;
# } Vector3;

# typedef struct _Quaternion {
#     float x;
#     float y;
#     float z;
#     float w;
# } Quaternion;