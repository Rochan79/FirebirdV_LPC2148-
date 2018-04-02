import serial       #pip3 install pyserial#
ser = serial.Serial('/dev/ttyACM0',9600)
import os
os.getcwd()
import time
file =open('samples.txt')  #can be changed to .csv
while 1:
   line = file.readline()
   if not line:
         break
   ser.write(line)
   time.sleep(3)
