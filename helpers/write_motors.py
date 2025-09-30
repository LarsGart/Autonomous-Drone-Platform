import sys
sys.path.append("/home/drone/Autonomous-Drone-Platform/drone")
from xiao import Xiao

xiao = Xiao()

# Main function to receive motor speeds and output them
def main():
   print(xiao)
   print('entering main')
   xiao._arm()
   while True:
      spd = input("Enter motor speed (0-2047): ")
      try:
         speedList = [int(x) for x in spd.split(",")]
      except ValueError:
         print("Invalid Speeds!")
         speedList = [0] * 4
      xiao._set_speeds(speedList)

if __name__ == '__main__':
   try:
      main()
   except KeyboardInterrupt:
      xiao._stop_motors()
      del xiao