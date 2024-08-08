import sys
sys.path.append("/home/drone/Autonomous-Drone-Platform/models")
from controller import Controller

if __name__ == '__main__':
    C = Controller()
    try:
        C.run()
    except KeyboardInterrupt:
        C.logger.info("Closing controller.")
        C.close()
        sys.exit(0)

