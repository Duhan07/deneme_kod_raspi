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
	
	
def position(pos_x, pos_y, yaw_rate, pos_z, iha):
	msg = iha.message_factory.set_position_target_local_ned_encode(
	 0,
	 0, 0,
	 mavutil.mavlink.MAV_FRAME_LOCAL_NED,
	 0b0000011111111000,
	 pos_x, pos_y, pos_z, #pozisyonlar (metre)
	 0, 0, 0, #hizlar (metre/s)
	 0, 0, 0, #akselarasyon (fonksiyonsuz)
	 0, math.radians(yaw_rate)) #yaw, yaw_rate (rad, rad/s)

	iha.send_mavlink(msg)
	
arm_ol_ve_yuksel(15)


position(5, 0, 0, -15, iha)

while iha.location.local_frame.north <= 4.9:
	print("5 metre kuzeye, 15 metre yukari ilerliyorum...")
	time.sleep(1)
print("5 metre kuzey, 15 metre yukaridayim.")
time.sleep(1)

position(5, 5, 0, -15, iha)
while iha.location.local_frame.east <= 4.9:	
	print("5 metre doguya ilerliyorum...")
	time.sleep(1)
print("5 metre dogudayim.")

position(0, 5, 0, -15, iha)
while iha.location.local_frame.north >= 0.1:
	print("5 metre geri(guneye) ilerliyorum...")
	time.sleep(1)
print("5 metre geri(guney)deyim.")

position(0, 5, 0, -25, iha)
while iha.location.local_frame.down >= -24.90:
	print("10 metre daha yukari ilerliyorum...")
	print("yukseklik: {}".format(iha.location.local_frame.down))
	time.sleep(1)
print("25 metre yukaridayim.")

iha.mode = VehicleMode("RTL")
print("Eve Donuyorum...")








