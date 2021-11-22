import socket

localIP     = "localhost"
localPort   = 44444

sock = socket.socket(family = socket.AF_INET, type = socket.SOCK_DGRAM)
sock.bind((localIP, localPort))

print("Listening for UDP packets...")
while(True):
    print(sock.recvfrom(1024)[0].decode())