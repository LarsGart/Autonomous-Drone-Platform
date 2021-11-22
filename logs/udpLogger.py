import socket

localIP     = "0.0.0.0"
localPort   = 44444

sock = socket.socket(family = socket.AF_INET, type = socket.SOCK_DGRAM)
sock.bind((localIP, localPort))


data = []
index = 0
print("Listening for UDP packets...")
while(True):
    stream = sock.recvfrom(1024)[0].decode()

    print(index)
    data.append(stream)
    if index == 600:
        break
    index+=1

textfile = open("dataDump.txt", "w")
print('Writing to dataDump.txt')
for element in data:
    textfile.write(str(element) + "\n")
textfile.close()