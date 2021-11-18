import socket
import time

msgFromClient = 'w0.09wa-0.12ab-0.09bc0.98c\n'
bytesToSend = str.encode(msgFromClient)
serverAddressPort = ('127.0.0.1',20001)
bufferSize = 1024

socc = socket.socket(family = socket.AF_INET, type = socket.SOCK_DGRAM)

while True:
	socc.sendto(bytesToSend, serverAddressPort)
	print(bytesToSend)
	time.sleep(1)