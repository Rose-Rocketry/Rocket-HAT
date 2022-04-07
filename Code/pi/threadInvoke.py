# Delete
import pickle
import serial
from IMULogging import *
from smbus import SMBus
import time
from datetime import datetime
from Quaternion import Quaternion

def startIMULoggerThread():
    IMULogger()

def startRFProccessingThread():                                                                                                                                                           
    RFLogging.main()