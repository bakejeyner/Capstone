while(1):
  speedString = raw_input("Please enter a speed beween -500 and 500...\n")
  
  if speedString.isdigit() == False:
    break
    
  speed = int(speedString)

  print("Speed of ", speed, "mm/sec\n")
  raw_input("Press Enter to Continue...\n")

print('Finished')
