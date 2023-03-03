import serial

uart2 = serial.Serial(
    port="/dev/ttyS0",
    baudrate=115200
)

# Split speeds into MSB and LSB for each speed and send byte stream to speed controller via UART2
def outputSpeeds(speeds):
    uart2.write(bytearray([
        60, # Sends a '<'
        (speeds[0] >> 8) & 255, speeds[0] & 255,
        (speeds[1] >> 8) & 255, speeds[1] & 255,
        (speeds[2] >> 8) & 255, speeds[2] & 255,
        (speeds[3] >> 8) & 255, speeds[3] & 255,
        62 # Sends a '>'
    ]))

def main():
	while 1:
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