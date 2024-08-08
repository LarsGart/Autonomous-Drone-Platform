import sys

sys.path.append("/home/drone/Autonomous-Drone-Platform/models")

from rx import RX

rx = RX()

def main():
    while True:
        print(rx.readRX())

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        del rx