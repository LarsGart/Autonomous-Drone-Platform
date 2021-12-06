from ahrs.filters import ROLEQ
import numpy as np
import socket

UDP_IP = "0.0.0.0"
UDP_PORT = 44444
sock = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

filt = ROLEQ()
q0 = np.array([1, 0, 0, 0])
while True:
	data = sock.recvfrom(1024)[0].decode()
	gx, gy, gz, ax, ay, az, mx, my, mz = np.float_(data.split(','))
	q = filt.update(
		q=q0,
		gyr=np.array([gx, gy, gz]) * 180 / np.pi,
		acc=np.array([ax, ay, az]),
		mag=np.array([mx, my, mz])
	)
	q0 = q
	print(q)