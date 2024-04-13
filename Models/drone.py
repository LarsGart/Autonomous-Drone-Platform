import sys
from time import sleep # I'm sleepy

sys.path.append("/home/drone/Autonomous-Drone-Platform/Models")

from flight_controller_model import FlightController

if __name__ == '__main__':
    flight_controller = FlightController(test_mode=True)
    try:
        flight_controller.logger.info("Entering Event Loop")
        flight_controller.run()
    except KeyboardInterrupt:
        flight_controller.logger.info("Received termination command")
        flight_controller.close() 