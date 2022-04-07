from smbus import SMBus
import time
from datetime import datetime
from Quaternion import Quaternion
import RF_thread
import data_analysis_pi
import pickle
import serial
import threading as thread
from keyboard import press

bus = SMBus(3)

isLanding = False
isLiftoff = False

liftedOff = "not lifted off"
landed = "not landed"

xbee = serial.Serial("/dev/serial0", baudrate = 115200)
packetsSent = 0

MPL_ADDR = 0x60
MPL_P = 0x01
MPL_CTRL_REG1 = 0x26

#H3L_ADDR = 0x18
BNO_ADDR = 0x28

#H3L_CTRL_REG1 = 0x20

BNO_OPR_MODE = 0x3D
BNO_OPR_MODE_NDOF = 0x0C
BNO_OPR_MODE_IMU = 0x08
BNO_LIA_DATA = 0x08 # XYZ, LSB first, 6 bytes
BNO_EUL_DATA = 0x1A # yaw roll pitch, Least significant register first, 6 bytes
BNO_QUA_DATA = 0x20 # wxyz, LSB, 8 bytes

bus.write_byte_data(MPL_ADDR, MPL_CTRL_REG1, 0x81) # Turn MPL on, alt mode
#bus.write_byte_data(H3L_ADDR, H3L_CTRL_REG1, 0x3F) # Turn the chip on, accel enabled, 1000Hz sample
bus.write_byte_data(BNO_ADDR, BNO_OPR_MODE, BNO_OPR_MODE_IMU) # Turn BNO on

altitude = 0
accel_p = Quaternion(0, 0, 0, 0)
ori_q = Quaternion(0, 0, 0, 0)
n = 0
AW = 0
AX = 0
AY = 0
AZ = 0
posX = 0
posY = 0
posZ = 0
velX = 0
velY = 0
velZ = 0

file = open("/home/pi/Rocket-HAT/datafiles/"+str(datetime.today().strftime('%Y-%m-%d_%H-%M-%S')) + ".csv", "w+")
file.write("Time,Altitude,Rotation w,Rotation x,Rotation Y,Rotation Z,Acceleration Qw,Acceleration Qx,Acceleration Qy,Acceleration Qz,Acceleration X,Acceleration Y,Acceleration Z,Position X,Position Y,Position Z\n")

landing_alt_thresh = 20 # 30 m
landing_time_thresh = 10 # 10 s
liftoff_accel_thresh = 15 # 20 m/s^2
liftoff_time_thresh = 0.2 # 0.2 seconds

def getIMUData():
    global altitude
    global accel_p
    global ori_q
    altitude_data = bus.read_i2c_block_data(MPL_ADDR, MPL_P, 3)
    altitude = int.from_bytes(altitude_data, 'big', signed=True)
    altitude = altitude / 256

#    highAccel_data = bus.read_i2c_block_data(H3L_ADDR, 0xA8, 6)
#    highAccelx_data = [highAccel_data[1], highAccel_data[0]]
#    highAccely_data = [highAccel_data[3], highAccel_data[2]]
#    highAccelz_data = [highAccel_data[5], highAccel_data[4]]
#    highAccelx = int.from_bytes(highAccelx_data, 'big', signed=True)
#    highAccely = int.from_bytes(highAccely_data, 'big', signed=True)
#    highAccelz = int.from_bytes(highAccelz_data, 'big', signed=True)
#    highAccel_q = Quaternion(0, highAccelx, highAccely, highAccelz)

    accel_data = bus.read_i2c_block_data(BNO_ADDR, BNO_LIA_DATA, 6)
    accelz_data = [accel_data[5], accel_data[4]]
    accely_data = [accel_data[3], accel_data[2]]
    accelx_data = [accel_data[1], accel_data[0]]
    accelx = (int.from_bytes(accelx_data, 'big', signed=True) - 1) / 100
    accely = (int.from_bytes(accely_data, 'big', signed=True) - 2) / 100
    accelz = (int.from_bytes(accelz_data, 'big', signed=True) + 25)/ 100
    accel_q = Quaternion(0, accelx, accely, accelz)

    # gyro_data = bus.read_i2c_block_data(BNO_ADDR, BNO_EUL_DATA, 6)
    # pitch_data = [gyro_data[5], gyro_data[4]]
    # roll_data = [gyro_data[3], gyro_data[2]]
    # yaw_data = [gyro_data[1], gyro_data[0]]
    # pitch = int.from_bytes(pitch_data, 'big', signed=True) / 16
    # roll = int.from_bytes(roll_data, 'big', signed=True) / 16
    # yaw = int.from_bytes(yaw_data, 'big', signed=True) / 16

    gyro_data = bus.read_i2c_block_data(BNO_ADDR, BNO_QUA_DATA, 8)
    qw_data = [gyro_data[1], gyro_data[0]]
    qx_data = [gyro_data[3], gyro_data[2]]
    qy_data = [gyro_data[5], gyro_data[4]]
    qz_data = [gyro_data[7], gyro_data[6]]
    qw = int.from_bytes(qw_data, 'big', signed=True) / 16384
    qx = int.from_bytes(qx_data, 'big', signed=True) / 16384
    qy = int.from_bytes(qy_data, 'big', signed=True) / 16384
    qz = int.from_bytes(qz_data, 'big', signed=True) / 16384
    ori_q = Quaternion(qw, qx, qy, qz)

    accel_p = ori_q * accel_q * ~ori_q

startTime = time.time()
prevTime = time.time()
currTime = time.time()
dTime = currTime - prevTime

messageSent = time.time()

while(currTime-startTime < 10):
    print("Initializing IMU data....")

    getIMUData()

    currTime = time.time()
    dTime = currTime - prevTime
    prevTime = currTime

    AW += accel_p.w
    AX += accel_p.x
    AY += accel_p.y
    AZ += accel_p.z
    n = n + 1

aw = AW/n
ax = AX/n
ay = AY/n
az = AZ/n

liftoffTime = time.time()
while(not isLiftoff):
    print("No liftoff")
    
    getIMUData()
    
    currTime = time.time()
    dTime = currTime - prevTime
    prevTime = currTime

    velX = velX + dTime * (accel_p.x-ax)
    velY = velY + dTime * (accel_p.y-ay)
    velZ = velZ + dTime * (accel_p.z-az)

    posX = posX + dTime * velX
    posY = posY + dTime * velY
    posZ = posZ + dTime * velZ
    file.write(str(currTime-startTime) + "," + str(altitude) + "," + str(ori_q.w) + "," + str(ori_q.x) + "," + str(ori_q.y) + "," + str(ori_q.z) + "," + str(accel_p.w-aw) + "," + str(accel_p.x-ax) + "," + str(accel_p.y-ay) + "," + str(accel_p.z-az) + "," + str(accel_p.x) + "," + str(accel_p.y) + "," + str(accel_p.z) + "," + str(posX) + "," + str(posY) + "," + str(posZ) + "\n")
    print(accel_p.z)
    if(accel_p.z-az < liftoff_accel_thresh):
        liftoffTime = time.time()
    if(time.time()-liftoffTime > liftoff_time_thresh):
        isLiftoff = True
    if (isLiftoff == True):
        liftedOff = "lifted off"
    if (isLanding == True):
        landed = "landed"
    if (isLiftoff == True):
        liftedOff = "not lifted off"
    if (isLanding == True):
        landed = "not landed"
    
    # Send data through XBee
    if ((time.time() - messageSent) >= 0.05):
        packetsSent = packetsSent + 1
        telemetry = { 'position': [str(posX), str(posY), str(altitude)], 'velocity': [str(velX), str(velY), str(velZ)], 'acceleration': [str(accel_p.w-aw), str(accel_p.x-ax), str(accel_p.y-ay), str(accel_p.z-az)], 'packets sent': str(packetsSent), 'status': [liftedOff, landed] }
        binary = pickle.dumps(telemetry)
        xbee.write(binary)
        xbee.write(b'STOPSTOPSTOPSTOP')
        messageSent = time.time()

landingTime = time.time()
landingAlt = altitude
landed = "not landed"
while (not isLanding):
    print("No landing")
    
    getIMUData()

    if(abs(landingAlt-altitude) > landing_alt_thresh):
        landingAlt = altitude
        landingTime = time.time()
    if(time.time()-landingTime > landing_time_thresh):
        isLanding = True

    currTime = time.time()
    dTime = currTime - prevTime
    prevTime = currTime

    velX = velX + dTime * (accel_p.x-ax)
    velY = velY + dTime * (accel_p.y-ay)
    velZ = velZ + dTime * (accel_p.z-az)

    posX = posX + dTime * velX
    posY = posY + dTime * velY
    posZ = posZ + dTime * velZ

    file.write(str(currTime-startTime) + "," + str(altitude) + "," + str(ori_q.w) + "," + str(ori_q.x) + "," + str(ori_q.y) + "," + str(ori_q.z) + "," + str(accel_p.w-aw) + "," + str(accel_p.x-ax) + "," + str(accel_p.y-ay) + "," + str(accel_p.z-az) + "," + str(accel_p.x) + "," + str(accel_p.y) + "," + str(accel_p.z) + "," + str(posX) + "," + str(posY) + "," + str(posZ) + "\n")

    if (isLiftoff == True):
        liftedOff = "lifted off"
    if (isLanding == True):
        landed = "landed"
    if (isLiftoff == True):
        liftedOff = "not lifted off"
    if (isLanding == True):
        landed = "not landed"
    
    # Send data through XBee
    if ((time.time() - messageSent) >= 0.05):
        packetsSent = packetsSent + 1
        telemetry = { 'position': [str(posX), str(posY), str(altitude)], 'velocity': [str(velX), str(velY), str(velZ)], 'acceleration': [str(accel_p.w-aw), str(accel_p.x-ax), str(accel_p.y-ay), str(accel_p.z-az)], 'packets sent': str(packetsSent), 'status': [liftedOff, landed] }
        binary = pickle.dumps(telemetry)
        xbee.write(binary)
        xbee.write(b'STOPSTOPSTOPSTOP')
        messageSent = time.time()

print("running RF_thread")
RF_thread.main()
print("running data analysis")
data_analysis_pi.main() # Start data_analysis_pi
