import socket
import argparse

# Define socket parameters
localIP = "0.0.0.0"
localPort = 44444

# Instantiate socket
sock = socket.socket(family = socket.AF_INET, type = socket.SOCK_DGRAM)
sock.bind((localIP, localPort))

# Define quaternion argument for cmd line
parser = argparse.ArgumentParser()
parser.add_argument("-q", "--quaternion", help = "Use if received data is a byte encoded quaternion", action = "store_true")

args = parser.parse_args()

print("Listening for UDP packets...")
while True:
    data, addr = sock.recvfrom(1024)
    if (args.quaternion):
        print([2/255 * b - 1 for b in data[:4]])
    else:
        print(data.decode())
