import socket
from joblib import dump, load
import os.path
from time import sleep
import numpy as np

localIP = "0.0.0.0"
localPort = 44444

sock = socket.socket(family = socket.AF_INET, type = socket.SOCK_DGRAM)
sock.bind((localIP, localPort))

homeDir = os.path.expanduser('~/Autonomous-Drone-Platform/logs/')
calibFile = homeDir + 'calibrationParams.joblib'

def sphereFit(spX,spY,spZ):
    # Assemble the A matrix
    spX = np.array(spX)
    spY = np.array(spY)
    spZ = np.array(spZ)
    A = np.zeros((len(spX),4))
    A[:,0] = spX*2
    A[:,1] = spY*2
    A[:,2] = spZ*2
    A[:,3] = 1
    # Assemble the f matrix
    f = np.zeros((len(spX),1))
    f[:,0] = (spX*spX) + (spY*spY) + (spZ*spZ)
    C, residules, rank, singval = np.linalg.lstsq(A,f)
    return C[0], C[1], C[2]

def calibrate():
    packet = sock.recvfrom(1024)[0].decode()
    magnetometerData = []

    while packet != 'end':
        packet = sock.recvfrom(1024)[0].decode()
        magnetometerData.append(packet)
        print(packet)
    magnetometerData = magnetometerData[:-1]

    x,y,z = [],[],[]

    for coord in magnetometerData:
        splitted = coord.split(',')
        x.append(float(splitted[0]))
        y.append(float(splitted[1]))
        z.append(float(splitted[2]))

    fittedX,fittedY,fittedZ = sphereFit(x,y,z)
    return fittedX,fittedY,fittedZ 

def calibrationQuery(ipaddr):
    fileExists = os.path.exists(calibFile)

    if fileExists:
        print('Calibration parameters found')
        calibrationParams = load(calibFile)
        sock.sendto(str.encode(calibrationParams), ipaddr)

    else:
        print('No calibration parameters found')
        sock.sendto(b'n', ipaddr)
        fittedX,fittedY,fittedZ = calibrate()

        calibrationParams = str(fittedX[0])+','+str(fittedY[0])+','+str(fittedZ[0])
        dump(calibrationParams, calibFile)
        print('Message is: ', calibrationParams, type(calibrationParams))
        sock.sendto(str.encode(calibrationParams), ipaddr)

print("Listening for UDP packets...")

while (True):
    data = sock.recvfrom(1024)
    packet = data[0].decode()

    if packet == 'calibration query':
        calibrationQuery(data[1])
    
    elif packet[0] == 'H':
        print(packet)
        break
