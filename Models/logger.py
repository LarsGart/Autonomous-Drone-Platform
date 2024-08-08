import logging
from datetime import datetime

class Logger:
    log_file = None

    @classmethod
    def initialize_logger(cls, log_file=None):
        if log_file is None:
            log_file = f"/home/drone/Autonomous-Drone-Platform/Logs/drone_{datetime.now().strftime('%m-%d_%H-%M-%S')}.log"
        
        cls.log_file = log_file
        logging.basicConfig(
            filename=cls.log_file,
            level=logging.DEBUG,
            format='%(asctime)s.%(msecs)03d,%(name)s,%(message)s',
            datefmt='%H:%M:%S'
        )

    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.log(f"{self.__class__.__name__} successfully initialized.")

    def log(self, message, level='INFO'):
        if level == 'DEBUG':
            self.logger.debug(message)
        elif level == 'INFO':
            self.logger.info(message)
        elif level == 'WARNING':
            self.logger.warning(message)
        elif level == 'ERROR':
            self.logger.error(message)
        elif level == 'CRITICAL':
            self.logger.critical(message)