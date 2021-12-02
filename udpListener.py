import socket

localIP     = "0.0.0.0"
localPort   = 44444

sock = socket.socket(family = socket.AF_INET, type = socket.SOCK_DGRAM)
sock.bind((localIP, localPort))

print("Listening for UDP packets...")
while(True):
    data = sock.recvfrom(1024)
    print(data[0].decode())
