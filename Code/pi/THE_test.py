import serial
import time
import pickle

xbee = serial.Serial("/dev/serial0", baudrate = 115200)

for i in range(1,20):
	if (i > 18):
		xbee.write(pickle.dumps({"status": "landed"}))
	else:
		xbee.write(pickle.dumps({"status": "not landed"}))
	time.sleep(0.5)
