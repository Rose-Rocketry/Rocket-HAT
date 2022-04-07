# Hey, don't init off this. thanmpk ypu

# Give it 20 sec for GS to start transmitting
# When MPL is not changing, 

import threading as thread
from IMULogging import *
from RF_thread import *

T_IMU = thread.Thread(target=TI.IMULogger())

T_RF = thread.Thread(target=RF_thread.main())

T_IMU.start()

T_IMU.join()
