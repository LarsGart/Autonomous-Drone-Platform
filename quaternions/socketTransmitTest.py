import socket
import time
import sys

serverAddressPort = ('127.0.0.1', 44444)
bufferSize = 1024

socc = socket.socket(family = socket.AF_INET, type = socket.SOCK_DGRAM)

from random import gauss

def make_rand_vector(dims):
    vec = [gauss(0, 1) for i in range(dims)]
    mag = sum(x**2 for x in vec) ** .5
    return [x/mag for x in vec]

def quatToBytes(quat):
	return bytes([int(127.5 * e + 127.5) for e in quat])

while True:
	quatBytes = quatToBytes(make_rand_vector(4))
	socc.sendto(quatBytes, serverAddressPort)
	time.sleep(0.05)