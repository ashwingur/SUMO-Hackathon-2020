# Importing the relevant libraries
import cv2
import numpy as np
import time
import os
import RPi.GPIO as GPIO
from picamera import PiCamera

# Setting the pin output for the buzzer
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
buzzer=23
GPIO.setup(buzzer,GPIO.OUT)


# Instantiating the camera object
camera = PiCamera()
camera.resolution=(1024,768)
camera.start_preview()

# Creating 4 arrays to store the scenery boundary boxes (if they are detected)
xarr = []
yarr = []
warr = []
harr = []


# The first frame taken by the camera is the calibration frame, which contains the stationary scenery. All objects detected in
# later frames are compared with the calibration frame to check for movement (eg a human entering the scene)
print("calibration frame")
forig = "calibration.png"
camera.capture(forig)

""" The following code is taken from the Hackathon openCV workshop demonstration. It is used to
perform segmentation on the frame. The end result is the boundary boxes found."""
img = cv2.imread(forig, flags=cv2.IMREAD_UNCHANGED)
H, W, C = img.shape

img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

mask = cv2.inRange(img_hsv, np.array([0,59,0]), np.array([25,255,114]))
mask = cv2.erode(mask, None, iterations=2)
mask = cv2.dilate(mask, None, iterations=2)

kernel_closing = np.ones((15,15),np.uint8)
kernel_opening = np.ones((11,11),np.uint8)

opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_opening)
closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel_closing)

edges = cv2.Canny(closing,100,200)
edges_og = cv2.Canny(img,100,200)

img, contours, _= cv2.findContours(edges, cv2.RETR_TREE,
                               cv2.CHAIN_APPROX_SIMPLE)



img_contours = img.copy()


for c in contours:
    random_colour = tuple(np.random.choice(range(256), size=3))
    cv2.drawContours(img_contours, [c], -1, (int(random_colour[0]), int(random_colour[1]), int(random_colour[2])), 5)


#in pixels!
min_area = 100
for c in contours:
   area = cv2.contourArea(c)
   if (area > min_area):
       moment = cv2.moments(c)
       cx = int(moment['m10']/(moment['m00'] + 1e-5))
       cy = int(moment['m01']/(moment['m00'] +  1e-5))
       x,y,w,h = cv2.boundingRect(c)

       cv2.rectangle(img,(x,y),(x+w,y+h),(255,255,255),4)
       cv2.circle(img, (cx, cy), 5, (255, 255, 255))

       # Adding the scenery object location and size into the relevant arrays
       xarr.append(x)
       yarr.append(y)
       warr.append(w)
       harr.append(h)

# Writing files to the current directory (For debugging purposes)
fboxx = "Cboxx.png"
fhsvv = "Chsvv.png"
fmask = "Cmask.png"
fclos = "Cclos.png"
fopen = "Copen.png"
fedge = "Cedge.png"
cv2.imwrite(fmask, mask)
cv2.imwrite(fclos, closing)
cv2.imwrite(fopen, opening)
cv2.imwrite(fboxx, img)
cv2.imwrite(fedge, edges)
cv2.imwrite(f, img_contours)
cv2.imwrite(fhsvv, img_hsv)

# Printing the scenery objects for debugging purposes
print(xarr, yarr, warr, harr)

i = 1

"""Running the camera forever until the user turns off the system. Some code is taken from the openCV workshop demo files """
while True:
    # Printing current frame for debug purposes
    print(f"frame: {i}")
    f = str(i) + ".png"
    forig = str(i) + "orig.png"
    camera.capture(forig)
    img = cv2.imread(forig, flags=cv2.IMREAD_UNCHANGED)
    H, W, C = img.shape

    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # The chosen threshold values from colour picker (using sample images)
    mask = cv2.inRange(img_hsv, np.array([0,53,47]), np.array([255,216,111]))
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    kernel_closing = np.ones((15,15),np.uint8)
    kernel_opening = np.ones((11,11),np.uint8)

    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_opening)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel_closing)

    edges = cv2.Canny(closing,100,200)
    edges_og = cv2.Canny(img,100,200)

    img, contours, _= cv2.findContours(edges, cv2.RETR_TREE,
                                   cv2.CHAIN_APPROX_SIMPLE)



    img_contours = img.copy()


    for c in contours:
        random_colour = tuple(np.random.choice(range(256), size=3))
        cv2.drawContours(img_contours, [c], -1, (int(random_colour[0]), int(random_colour[1]), int(random_colour[2])), 5)

    # Error margin for detecting a scenery object
    err = 100

    #in pixels!
    min_area = 100
    for c in contours:
       area = cv2.contourArea(c)
       if (area > min_area):
           moment = cv2.moments(c)
           cx = int(moment['m10']/(moment['m00'] + 1e-5))
           cy = int(moment['m01']/(moment['m00'] +  1e-5))

           x,y,w,h = cv2.boundingRect(c)

           # Printing the rectangle for debug
           print(f"{x},{y},{w},{h}")

           # Define/reset the isScenery flag. Objects that enter the frame are not classified as scenery as they were not in the calibration frame
           isScenery = False

           # Comparing the current object in the frame to known scenery objects to check if it's a scenery object. If it is, then the isScenery flag is true,
           # and we go onto the next object.
           for j in range(0,len(xarr)):
               if x >= xarr[j] - err and x<= xarr[j] + err:
                   if y >= yarr[j] - err and y<= yarr[j] + err:
                       if w >= warr[j] - err and w<= warr[j] + err:
                           if h >= harr[j] - err and h<= harr[j] + err:
                               isScenery = True
                               break



            # If the object is not in the scenery then we raise the alarm that a human has been detected (if the object is greater than a certain size so things like animals
            # do not cause false alarms)
           if isScenery == False:
               if h > 15 and w > 15:
                   print("Human")

                   # Sound the alarm!
                   for k in range(0,750):
                        GPIO.output(buzzer,GPIO.HIGH)
                        time.sleep(0.002) # Delay in seconds
                        GPIO.output(buzzer,GPIO.LOW)
                        time.sleep(0.002)

                   cv2.rectangle(img,(x,y),(x+w,y+h),(255,255,255),4)
                   cv2.circle(img, (cx, cy), 5, (255, 255, 255))

                   break


    # For debugging
    fboxx = str(i) + "boxx.png"
    #fhsvv = str(i) + "hsvv.png"
    #fmask = str(i) + "mask.png"
    #fclos = str(i) + "clos.png"
    #fopen = str(i) + "open.png"
    #fedge = str(i) + "edge.png"
    #cv2.imwrite(fmask, mask)
    #cv2.imwrite(fclos, closing)
    #cv2.imwrite(fopen, opening)
    cv2.imwrite(fboxx, img)
    #cv2.imwrite(fedge, edges)
    #cv2.imwrite(f, img_contours)
    #cv2.imwrite(fhsvv, img_hsv)
    time.sleep(2)

    i += 1



os.system("pkill python3")
exit()
