import sys
from time import sleep # I'm sleepy
import unittest
import glob
from pathlib import Path

sys.path.append("/home/drone/Autonomous-Drone-Platform/Models")

from flight_controller_model import FlightController

def run_tests(flight_controller):
    test_files = glob.glob("Unit_Tests/test_*.py")
    module_strings = [Path(test_file).stem for test_file in test_files]
    suites = [unittest.defaultTestLoader.loadTestsFromName(test_file) for test_file in module_strings]
    test_suite = unittest.TestSuite(suites)
    test_runner = unittest.TextTestRunner().run(test_suite)
    
    if test_runner.wasSuccessful():
        flight_controller.logger.info("All tests passed")
        return True

    flight_controller.logger.error("Some tests failed")
    flight_controller.close()
    return False

def main():
    flight_controller = FlightController()

    if len(sys.argv) > 1 and sys.argv[1] == "test" and not run_tests(flight_controller):
        sys.exit("Tests failed. Exiting.")

    flight_controller.logger.info("Entering Event Loop")
    try:
        flight_controller.run()
    except KeyboardInterrupt:
        flight_controller.logger.info("Received termination command")
        flight_controller.close()

if __name__ == '__main__':
    main()
