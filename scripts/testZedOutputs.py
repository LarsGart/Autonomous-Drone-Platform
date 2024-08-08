import sys

sys.path.append("/home/drone/Autonomous-Drone-Platform/models")

from zed import Zed

def main():
    zed = Zed()
    p0 = zed.get_pos_relative()
    print(p0)
    zed.close()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Program Closed')