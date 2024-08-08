import sys
from models.controller import Controller
from models.logger import Logger

if __name__ == '__main__':
    Logger.initialize_logger()
    C = Controller()
    try:
        C.run()
    except KeyboardInterrupt:
        C.logger.info("Closing controller.")
        C.close()
        sys.exit(0)

