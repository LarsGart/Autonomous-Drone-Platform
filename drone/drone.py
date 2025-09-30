from controller import Controller
from receiver import Receiver
from zed import Zed


zed = Zed()
controller = Controller(zed)
receiver = Receiver()

if __name__ == "__main__":
    while True:
        try:
            receiver_read = receiver._read_normalized()
            controller._update(
                throttle_pwm=receiver_read.throttle,
                stick_roll=receiver_read.roll,
                stick_pitch=receiver_read.pitch,
                stick_yaw=receiver_read.yaw
            )
        except KeyboardInterrupt:
            zed._close()
