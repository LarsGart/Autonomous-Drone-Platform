import socket

# Define UDP address and port
udpAddrPort = ('10.0.0.20', 44444)

# Instantiate socket
sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

print("Enter PID parameter to change and float value to change it to")
print("Format is (p,i,d) followed by (1,2,3) = (float)")
print("e.g. p2=0.5 or i3=0.01 or d1=10.0")
while 1:
    rawInput = input("Enter param: ")
    if (rawInput == chr(27)):
        sock.close()
        break

    sock.sendto(str.encode(rawInput), udpAddrPort)