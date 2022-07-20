from dronekit import connect, VehicleMode
import time

connection_string = "127.0.0.1:14550"

iha = connect(connection_string, wait_ready=True)



def beArm_and_up(target_alt):
	#arget_alt = int(input('Yukseklik giriniz: '))

	while iha.is_armable==False:
		print("Arm ici gerekli sartlar saglanamadi.")
		time.sleep(1)
		iha.armed = True
	print("iha su anda arm edilebilir.")
	
	iha.mode = VehicleMode("GUIDED")
	while iha.mode!='GUIDED':
		print("Guided moduna gecis yapiliyor...")
		time.sleep(1.5)
		if iha.mode == 'GUIDED':
			continue
		
	print("Guided moduna gecis yapildi.")
	iha.armed = True
	while iha.armed is False:
		print("Arm icin bekleniyor...")
		time.sleep(1)
		
	print("Ihamiz Arm olmustur.")
	
	iha.simple_takeoff(target_alt)
	while iha.location.global_relative_frame.alt <= target_alt*0.95:
		print("Su anki yukseklik: {}".format(iha.location.global_relative_frame.alt))
		time.sleep(0.5)
		
	print("Takeoff gerceklesti.")
	
	
beArm_and_up(40)
	
	
			
