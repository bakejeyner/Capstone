import serial
import time
  
ser = serial.Serial('/dev/ttyUSB0')
ser.baudrate = 57600

straight = 32768

ser.write(chr(128))
time.sleep(1)
ser.write(chr(130))
time.sleep(1)
ser.write(chr(132))
time.sleep(1)

while(1):
  speedString = raw_input("Please enter a speed beween -500 and 500...\n")
  
  if speedString.isdigit() == False:
    break
    
  speed = int(speedString)

  print("Speed of ", speed, "mm/sec\n")
  ser.write(chr(137))
  time.sleep(1)
  ser.write(chr((speed >> 8) & 255))
  time.sleep(1)
  ser.write(chr(speed & 255))
  time.sleep(1)
  ser.write(chr((straight >> 8) & 255))
  time.sleep(1)
  ser.write(chr(straight & 255))
  raw_input("Press Enter to Continue...\n")
  
ser.write(131)

print('Finished')
