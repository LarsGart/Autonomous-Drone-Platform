from orientationSensor import OrientationSensor

def main():
    sens = OrientationSensor('/dev/i2c-0')
    while 1:
        data = sens.readSensor()
        print(data[:3])

if __name__ == '__main__':
    main()