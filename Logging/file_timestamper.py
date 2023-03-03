'''
Method for providing timestamped filenames for log files.
'''
import datetime


def get_timestamped_filename(model_name: str):
    return '/Logging/' + model_name + '.log' \
        + datetime.datetime.now().strftime("%Y-%m-%d_%H-%M")
        