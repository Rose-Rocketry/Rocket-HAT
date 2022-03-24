# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
import adafruit_bno055
import adafruit_lis331
import adafruit_mpl3115a2

i2c = board.I2C(19, 20)
bno = adafruit_bno055.BNO055_I2C(i2c, address = )
lis = adafruit_lis331.H3LIS331(i2c, address = )
mpl = adafruit_mpl3115a2.MPL3115A2(i2c, address = )
mpl.sealevel_pressure = 101260
last_val = 0xFFFF

def temperature():
    global last_val  # pylint: disable=global-statement
    result = bno.temperature
    if abs(result - last_val) == 128:
        result = bno.temperature
        if abs(result - last_val) == 128:
            return 0b00111111 & result
    last_val = result
    return result

while True:
    print("Temperature: {} degrees C".format(bno.temperature))
    """
    print(
        "Temperature: {} degrees C".format(temperature())
    )  # Uncomment if using a Raspberry Pi
    """
    print("Accelerometer (m/s^2): {}".format(bno.acceleration))
    print("Magnetometer (microteslas): {}".format(bno.magnetic))
    print("Gyroscope (rad/sec): {}".format(bno.gyro))
    print("Euler angle: {}".format(bno.euler))
    print("Quaternion: {}".format(bno.quaternion))
    print("Linear acceleration (m/s^2): {}".format(bno.linear_acceleration))
    print("Gravity (m/s^2): {}".format(bno.gravity))
    print("H3LIS331 Acceleration : X: %.2f, Y:%.2f, Z:%.2f ms^2" % lis.acceleration)
    print("Pressure: {0:0.3f} pascals".format(mpl.pressure))
    print("Altitude: {0:0.3f} meters".format(mpl.altitude))
    print("Temperature: {0:0.3f} degrees Celsius".format(mpl.temperature))
    print()

    time.sleep(1.0)
