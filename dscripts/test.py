from __future__ import print_function
from dronekit import connect, VehicleMode

v = connect('/dev/ttyACM0', wait_ready=True, baud=57600)

print(v.gps_0)
print("ekf_ok = ", v.ekf_ok)
print("is_armable = ", v.is_armable)  
print(v.location.global_relative_frame)


