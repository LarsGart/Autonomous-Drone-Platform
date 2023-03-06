import serial

# Define UART
uart2 = serial.Serial(port="/dev/ttyS0", baudrate=115200)

# Function to output speeds
def outputSpeeds(speeds):
    # Create byte array to send to speed controller via UART
    byte_arr = bytearray([60] + [(speeds[i] >> 8) & 255 for i in range(4)] + [speeds[i] & 255 for i in range(4)] + [62])
    uart2.write(byte_arr)

# Main function to receive motor speeds and output them
def main():
    while True:
        spd = input("Enter motor speed: ")
        try:
            speedList = [int(x) for x in spd.split(",")]
        except ValueError:
            print("Invalid Speeds!")
            speedList = [1000] * 4
        outputSpeeds(speedList)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        outputSpeeds([1000] * 4)
        