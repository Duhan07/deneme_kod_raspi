from dronekit import connect, VehicleMode
import time 
from pymavlink import mavutil
import math

connection_string = "127.0.0.1:14550"

iha = connect(connection_string, wait_ready=True, timeout=100)

def arm_ol_ve_yuksel(hedef_yukseklik):
	while iha.is_armable == False:
		print("Arm ici gerekli sartlar saglanamadi.")
		time.sleep(1)
	print("Iha su anda arm edilebilir.")
	
	iha.mode = VehicleMode("GUIDED")
	while iha.mode == 'GUIDED':
		print("Guided moduna gecis yapiliyor...")
		time.sleep(1.5)
	print("Guided moduna gecis yapildi.")
	
	iha.armed = True
	while iha.armed is False:
		print("Arm icin bekleniliyor..")
		time.sleep(1)
	print("Ihamzi arm olmustur.")
	
	iha.simple_takeoff(hedef_yukseklik)
	while iha.location.global_relative_frame.alt <= hedef_yukseklik*0.94:
		print("Su anki yukselik: {}".format(iha.location.global_relative_frame.alt))
		time.sleep(0.5)
	print("Takeoff gerceklesti.")
	
	
def velocity(velocity_x, velocity_y, yaw_rate, velocity_z, iha):
	msg = iha.message_factory.set_position_target_local_ned_encode(
	 0,
	 0, 0,
	 mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
	 0b0000011111000111,
	 0, 0, 0,
	 velocity_x, velocity_y, velocity_z,
	 0, 0, 0,
	 0, math.radians(yaw_rate))

	iha.send_mavlink(msg)
	
arm_ol_ve_yuksel(20)

velocity(5, 0, 0, 0, iha)
print("x ekseninde ilerliyor..")
time.sleep(2)

velocity(0, 5, 0, 0, iha)
print("y ekseninde ilerliyor..")
time.sleep(2)

velocity(0, 0, 0, -2, iha)
print("z ekseninde ilerliyor..")
time.sleep(2)

velocity(0, 0, 60, 0, iha)
print("Drone dönüyor..")
time.sleep(2)

velocity(0, 0, 0, 0, iha)


iha.mode = VehicleMode("LAND")










		
		
