import numpy as np
import cv2
from time import sleep
import sys
from pymavlink import mavutil

#### PYMAVLINK COMMAND
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600)
master.wait_heartbeat()


# Kırmızı renk aralığı(HSV)
redLower = (160, 5, 5)
redUpper = (179, 255, 255)

# Kırmızı renk aralığı(LAB)
redLower_lab = np.array([20, 150, 110])
redUpper_lab = np.array([190, 255, 255])


cap = cv2.VideoCapture(0)

tur = 0
while(True):
    # Capture frame-by-frame
    ret, captured_frame = cap.read()
    output_frame = captured_frame.copy()

    height_image = output_frame.shape[0]
    width_image = output_frame.shape[1]


    # Convert original image to BGR, since Lab is only available from BGR
    captured_frame_bgr = cv2.cvtColor(captured_frame, cv2.COLOR_BGRA2BGR)
    # First blur to reduce noise prior to color space conversion
    captured_frame_bgr = cv2.medianBlur(captured_frame_bgr, 3)
    # Convert to Lab color space, we only need to check one channel (a-channel) for red here
    captured_frame_lab = cv2.cvtColor(captured_frame_bgr, cv2.COLOR_BGR2Lab)
                                #HSV
    captured_frame_hsv = cv2.cvtColor(captured_frame_bgr, cv2.COLOR_BGR2HSV)

    # Threshold the Lab image, keep only the red pixels
    # Possible yellow threshold: [20, 110, 170][255, 140, 215]
    # Possible blue threshold: [20, 115, 70][255, 145, 120]
    captured_frame_lab_red = cv2.inRange(captured_frame_lab, np.array([20, 150, 110]), np.array([190, 255, 255]))
                                #HSV
    captured_frame_hsv_red = cv2.inRange(captured_frame_hsv, redLower, redUpper)

    # Second blur to reduce more noise, easier circle detection
    captured_frame_lab_red = cv2.GaussianBlur(captured_frame_lab_red, (5, 5), 2, 2)
                                #HSV
    captured_frame_hsv_red = cv2.GaussianBlur(captured_frame_hsv_red, (5, 5), 2, 2)

    # Use the Hough transform to detect circles in the image
    circles = cv2.HoughCircles(captured_frame_hsv_red, cv2.HOUGH_GRADIENT, 1, captured_frame_hsv_red.shape[0] / 8, param1=50, param2=30, minRadius=35, maxRadius=60)
    #print(captured_frame_lab_red.shape[0])
    if circles is not None:
        print(circles)
    cv2.line(output_frame, (0,int(np.round(height_image/2))), (int(np.round(width_image)), int(np.round(height_image/2))),color=(0,255,0),thickness=1)
    cv2.line(output_frame, (int(np.round(width_image/2)),0), (int(np.round(width_image/2)), int(np.round(height_image))),color=(0,255,0),thickness=1)


	# If we have extracted a circle, draw an outline
	# We only need to detect one circle here, since there will only be one reference object
    
    if circles is not None:
        
        
        circles = np.round(circles[0, :]).astype("int")
        height_circles = circles[0,1]
        width_circles = circles[0,0]
        
        cv2.circle(output_frame, center=(circles[0, 0], circles[0, 1]), radius=circles[0, 2], color=(0, 0, 255), thickness=2)

        print("image width center: ", np.round(width_image/2))
        print("image height center: ", np.round(height_image/2))

        print("circle height: ", height_circles)
        tur += 1
        print(tur)
        if tur == 1:
            print("Kırmızı alan tanındı (ilk tur)..")

            sleep(2)
        
        elif tur == 2:
            print("Birinci top birakildi (ikinci tur)..")

            servo1.start(0)
            time.sleep(2)

            duty = 2
            while duty <= 12:
                #servo1.ChangeDutyCycle(duty)
                time.sleep(1)
                #duty = duty + 1
                
            time.sleep(2)

            print("Turning back to 90 degrees for 2 seconds")
            #servo1.ChangeDutyCycle(2)
            #time.sleep(2)

            #servo1.ChangeDutyCycle(2)
            #time.sleep(0.5)
            #servo1.ChangeDutyCycle(0)

            #servo1.stop()
            #GPIO.cleanup()
            
            #sleep(2)

            
        elif tur == 3:
            print("İkinci top birakildi (ikinci tur)..")

            sleep(2)
            
        else:
            break
            
        
        
        print("Lat: ", Lat)
        print("Lng: ", Lng)
        
        if height_circles-20 < np.round(height_image/2) <= height_circles+20 and width_circles-20 < np.round(width_image/2) <= width_circles+20:
                
    
            cv2.circle(output_frame, center=(circles[0, 0], circles[0, 1]), radius=circles[0, 2], color=(0, 255, 0), thickness=2)
            print("Done")
            break
            

    # Display the resulting frame, quit with q
    cv2.imshow('frame', output_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
