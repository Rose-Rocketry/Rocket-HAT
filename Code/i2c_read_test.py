# /etc/init.d/i2c_read_test.py
### BEGIN INIT INFO
# Provides:          i2c_read_test.py
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: Start daemon at boot time
# Description:       Enable service provided by daemon.
### END INIT INFO

from smbus import SMBus
import time
from datetime import datetime
from Quaternion import Quaternion

MPL_ADDR = 0x60
H3L_ADDR = 0x18
BNO_ADDR = 0x28

MPL_P = 0x01
MPL_CTRL_REG1 = 0x26

H3L_CTRL_REG1 = 0x20

BNO_OPR_MODE = 0x3D
BNO_OPR_MODE_NDOF = 0x0C
BNO_OPR_MODE_IMU = 0x08
BNO_LIA_DATA = 0x08 # XYZ, LSB first, 6 bytes
BNO_EUL_DATA = 0x1A # yaw roll pitch, Least significant register first, 6 bytes
BNO_QUA_DATA = 0x20 # wxyz, LSB, 8 bytes


bus = SMBus(3)




# Get the H3L goin
bus.write_byte_data(H3L_ADDR, H3L_CTRL_REG1, 0x3F) # Turn the chip on, accel enabled, 1000Hz sample
bus.write_byte_data(MPL_ADDR, MPL_CTRL_REG1, 0x81) # Turn MPL on, alt mode
bus.write_byte_data(BNO_ADDR, BNO_OPR_MODE, BNO_OPR_MODE_IMU) # Turn BNO on

posX =0
posY=0
posZ=0
velX = 0
velY = 0
velZ = 0

startTime = time.time()
prevTime = time.time()
currTime = time.time()
dTime = currTime - prevTime

file = open("/home/pi/Rocket-HAT/datafiles/"+str(datetime.today().strftime('%Y-%m-%d_%H-%M-%S')) + ".csv", "w+")
file.write("Time,Altitude,Rotation w,Rotation x,Rotation Y,Rotation Z,Acceleration Qw,Acceleration Qx,Acceleration Qy,Acceleration Qz,Acceleration X,Acceleration Y,Acceleration Z,High Acceleration X,High Acceleration Y,High Acceleration Z,Position X,Position Y,Position Z\n")

while(1):
    # for i in range(1000000):
    #     pass

    altitude_data = bus.read_i2c_block_data(MPL_ADDR, MPL_P, 3)
    altitude = int.from_bytes(altitude_data, 'big', signed=True)
    altitude = altitude / 256

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

    highAccel_data = bus.read_i2c_block_data(H3L_ADDR, 0xA8, 6)
    highAccelx_data = [highAccel_data[1], highAccel_data[0]]
    highAccely_data = [highAccel_data[3], highAccel_data[2]]
    highAccelz_data = [highAccel_data[5], highAccel_data[4]]
    highAccelx = int.from_bytes(highAccelx_data, 'big', signed=True)
    highAccely = int.from_bytes(highAccely_data, 'big', signed=True)
    highAccelz = int.from_bytes(highAccelz_data, 'big', signed=True)
    

    accel_p = ori_q * accel_q * ~ori_q

    #print(ax)
    #print(accel_p.x)
    

    currTime = time.time()
    dTime = currTime - prevTime
    prevTime = currTime

    velX = velX + dTime * accel_p.x
    velY = velY + dTime * accel_p.y
    velZ = velZ + dTime * accel_p.z

    posX = posX + dTime * velX
    posY = posY + dTime * velY
    posZ = posZ + dTime * velZ

    file.write(str(currTime-startTime) + "," + str(altitude) + "," + str(qw) + "," + str(qx) + "," + str(qy) + "," + str(qz) + "," + str(accel_p.w) + "," + str(accel_p.x) + "," + str(accel_p.y) + "," + str(accel_p.z) + "," + str(accelx) + "," + str(accely) + "," + str(accelz) + "," + str(highAccelx) + "," + str(highAccely) + "," + str(highAccelz) + "," + str(posX) + "," + str(posY) + "," + str(posZ) + "\n")




file.close()
