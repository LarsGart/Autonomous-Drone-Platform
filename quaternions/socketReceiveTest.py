import socket


def main():
	UDP_IP = "127.0.0.1"
	UDP_PORT = 20001
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
	sock.bind((UDP_IP, UDP_PORT))
	print(sock)

	while True:
		data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
		line = data.decode('UTF-8').replace('\n', '')
		print(line)

if __name__ == '__main__':
    main()
