from time import sleep
import sys
from pymavlink import mavutil

#### PYMAVLINK COMMAND
master = mavutil.mavlink_connection('/dev/ttyAMA0')
master.wait_heartbeat()
# Request all parameters
#master.mav.param_request_list_send(
#    master.target_system, master.target_component
#)

#while True:
#    time.sleep(0.01)
#    try:
#        message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
#        print('name: %s\tvalue: %d' % (message['param_id'],
                                       message['param_value']))
        
        ## Deneme Konum Alma Kodu
#        lat = message['GPS_LATITIUDE'] # örnek
#        lng = message['GPS_LONGITIUDE'] # örnek
        
#    except Exception as error:
#        print(error)
#        sys.exit(0)


######## SERVO CONTROL IN RASPİ
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(11,GPIO.OUT)
servo1 = GPIO.PWM(11,50)

servo1.start(0)
time.sleep(2)

duty = 2
while duty <= 12:
    servo1.ChangeDutyCycle(duty)
    time.sleep(1)
    duty = duty + 1
    
time.sleep(2)

print("Turning back to 90 degrees for 2 seconds")
servo1.ChangeDutyCycle(2)
time.sleep(2)

servo1.ChangeDutyCycle(2)
time.sleep(0.5)
servo1.ChangeDutyCycle(0)

servo1.stop()
GPIO.cleanup()
print("BYE")
