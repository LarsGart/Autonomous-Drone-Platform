import socket

# Define udp address and port
udpAddrPort = ('10.0.0.22', 44444)

# Instantiate socket
sock = socket.socket(family = socket.AF_INET, type = socket.SOCK_DGRAM)

while True:
	msg = input("Enter data: ")
	if (msg == chr(27)): # If user inputs an esc, end the program
		break
	sock.sendto(str.encode(msg), udpAddrPort)