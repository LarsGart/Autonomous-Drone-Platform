from controller import Controller
from receiver import Receiver


controller = Controller()
receiver = Receiver()

if __name__ == "__main__":
    while True:
        receiver_read = receiver._read_normalized()
        controller._update(
            throttle_pwm=receiver_read.throttle,
            stick_roll=receiver_read.roll,
            stick_pitch=receiver_read.pitch,
            stick_yaw=receiver_read.yaw
        )
