from dronekit import connect
import time
v  = connect('/dev/ttyACM0', wait_ready=True, baud=57600)
while(True):
	print v.gps_0
	print('alt: ',v.location.global_relative_frame.alt)
	print('ekf: ',v.ekf_ok)
	time.sleep(5)
