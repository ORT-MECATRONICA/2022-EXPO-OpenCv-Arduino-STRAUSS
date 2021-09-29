import cv2 as cv
import serial
import numpy as np
import time
import math

valorCalibracion1 = 180
valorCalibracion2 = 0

laser_valueX = 0
laser_valueY = 0

capture = cv.VideoCapture(0, cv.CAP_DSHOW)
haar_cascade = cv.CascadeClassifier("FaceDetection/haar_faceee.xml")
# serialArduino = serial.Serial("COM7",9600) #Descomentar cuando haya algo en el puerto

def laser(frame):
    global laser_valueX, laser_valueY

    lower_red = np.array([0,0,255]) 
    upper_red = np.array([220,220,255]) 

    kernel = np.ones((3,3),np.uint8) # no se para que es

    mask_HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(mask_HSV, lower_red, upper_red) 
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    #mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
    output_mask = cv.bitwise_and(frame, frame, mask = mask)

    (minVal, maxVal,minLoc, maxLoc) = cv.minMaxLoc(mask) 
    cv.circle(frame, maxLoc, 20, (0, 0, 255), 2, cv.LINE_AA)

    output_hough = cv.cvtColor(output_mask, cv.COLOR_BGR2GRAY) 
    output_hough = cv.medianBlur(output_hough, 5)
    circles = cv.HoughCircles(output_hough, cv.HOUGH_GRADIENT, 1, 20, param1=18, param2=8, minRadius=0, maxRadius=15)
 
    #Circulo verde
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            cv.circle(frame,(i[0],i[1]),i[2],(0,255,0),2)
            cv.circle(frame,(i[0],i[1]),2,(0,255,0),3)

            laser_valueX = i[0]
            laser_valueY = i[1]

def rescaleFrame(frame, scale=0.75):  # Rescalar el video (Default = 0.75) 
    width = int(frame.shape[1]  * scale)
    height = int(frame.shape[0] * scale)

    dimensions = (width,height)

    return cv.resize(frame, dimensions, interpolation=cv.INTER_AREA) # Que es el INTER_AREA ???

while True:
    isTrue, frame1 = capture.read(0)
    frame1 = cv.flip(frame1, 1)
    frame1 = rescaleFrame(frame1, 1) #cambiar el valr aca para aumentar o achicar
    frame2 = rescaleFrame(frame1, 1)
    gray = cv.cvtColor(frame2, cv.COLOR_BGR2GRAY)
    #cv.imshow("Video2", gray)

    width = int(frame1.shape[1])
    height = int(frame1.shape[0])

    faces_rect = haar_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=3)

    for (x,y,w,h) in faces_rect:
        x_value = int(w / 5)
        y_value = int(h / 5)
        x_val = int(x + (w/2))
        y_val = int(y + (h/2))

        cv.rectangle(frame2, (x,y), (x+w,y+h), (0,255,0), thickness=2)
        laser(frame2)
        if (laser_valueX > x+x_value and laser_valueX < x+w-x_value and laser_valueY > y+y_value and laser_valueY < y+h-y_value): # Dentro del cuadrado chico
            cv.rectangle(frame2, (x+x_value,y+y_value), (x+w-x_value,y+h-y_value), (0,255,255), thickness=2)
        else:
            cv.rectangle(frame2, (x+x_value,y+y_value), (x+w-x_value,y+h-y_value), (0,255,0), thickness=2)
           
            # Falta poner los movimientos en serial
            if (laser_valueX < x+x_value):
                # Mover L (Revisar)
                pass
            elif (laser_valueX > x+w-x_value):
                # Mover R (Revisar)
                pass
            if (laser_valueY < y+y_value):
                # Mover D
                pass
            elif (laser_valueY > y+h-y_value):
                # Mover U
                pass

            # Poner lascondiciones paa enviar por serial.
  

    numpy_horizontal = np.hstack((frame2, frame1))
    cv.imshow('FaceDetection', numpy_horizontal)

    if cv.waitKey(19) & 0xFF==ord("f"):  # Tecla "f" para romper el while
        break

capture.release()
cv.destroyAllWindows()
