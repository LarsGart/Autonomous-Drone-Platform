import sys
from time import sleep # I'm sleepy

sys.path.append("/home/drone/Autonomous-Drone-Platform/Models")

from flight_controller_model import FlightController

if __name__ == '__main__':
    flight_controller = FlightController()
    try:
        print('trying run')
        flight_controller.run()
    except KeyboardInterrupt:
        flight_controller.logger.info("Received termination command")
        flight_controller.close() 
