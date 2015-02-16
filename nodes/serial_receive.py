#!/usr/bin/env python
import serial
import time
ser=serial.Serial('/dev/ttyUSB0', 57600, timeout=1)
for i in range(1,10000):
   line=ser.read(5)
   print line
   time.sleep(1)
ser.close()
