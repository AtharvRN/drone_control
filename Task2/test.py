from control_class import Pluto
import time
command = Pluto("192.168.4.1")

for i in range(10):
    command.disarm()
    time.sleep(0.1)

command.takeoff()
for i in range(30):
    command.set_attitude(throttle=1500,yaw = 1500,pitch= 1510,roll= 1510)
    time.sleep(0.1)

# for i in range(10):
#     command.disarm()
#     time.sleep(0.1)

command.land()
