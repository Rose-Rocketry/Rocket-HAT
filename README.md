# Rocket-HAT
A Sensor Hat for a raspberry pi with: an IMU, Altimeter, and 5V power protection

# Usage
There are two jumpers on the board; J7 is for powering the hat through the PI. Otherwise, the HAT will get power from the battery while also powering the Pi. J9 is to
enable writing to the EEPROM after manufacture. This EEPROM is for matching with the Raspberry Pi foundation's specs.

There is also a solder bridge for the BNO055. This is in case we have an address conflict and need to change the address from 0x28 to 0x29.
