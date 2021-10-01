# Falta hacer version para objetos (con colores y me ahorro todo lo de las caras)
import cv2 as cv
# import serial
import numpy as np
# import time
# import math

laser_valueX = 0
laser_valueY = 0

# Valores posibles: U (Up), D (Down), R (Right), L (Left), S (Stop)
send_serialX = "S"
send_serialY = "S"
actual_send_serialX = ""
actual_send_serialY = ""

capture = cv.VideoCapture(0, cv.CAP_DSHOW) # 0 para la camara default
haar_cascade = cv.CascadeClassifier("FaceDetection/haar_faceee.xml") # pPner ubicacion del archivo (Me genero problemas)
# Este archivo es para la deteccion de caras.

# serialArduino = serial.Serial("COM7",9600) #Descomentar cuando haya algo en el puerto

def laser(frame):
    global laser_valueX, laser_valueY

    # Valores Para la identificacion del laser (Ajustar segun iluminacion)
    lower_red = np.array([0,0,255])
    upper_red = np.array([220,220,255]) 

    mask_HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(mask_HSV, lower_red, upper_red) # Aca busca segun el rango de colores
    cont, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cv.drawContours(frame, cont, -1, (0, 0, 255), 4)

    for c in cont:
        area = cv.contourArea(c)
        if area > 6000: # Cambiar
            M = cv.moments(c)  # Ni idea que hace (Lo saque de un foro)
            if M["m00"] == 0:
                M["m00"] = 1

            X = int(M["m10"] / M["m00"])
            Y = int(M["m01"] / M["m00"])

            cv.circle(frame, (X,Y), 5, (0, 255, 0), -1)
            newCont = cv.convexHull(c)
            cv.drawContours(frame,  [newCont], 0, (0, 0, 255), 3)

def rescaleFrame(frame, scale=0.75):  # Rescalar el video (Default = 0.75) 
    width = int(frame.shape[1]  * scale)
    height = int(frame.shape[0] * scale)

    dimensions = (width,height)

    return cv.resize(frame, dimensions, interpolation=cv.INTER_AREA)


while True:
    isTrue, frame1 = capture.read(0)
    frame1 = cv.flip(frame1, 1)
    frame1 = rescaleFrame(frame1, 1) # Cambiar el valor  para aumentar o achicar
    frame2 = rescaleFrame(frame1, 1)
    gray = cv.cvtColor(frame2, cv.COLOR_BGR2GRAY)
    #cv.imshow("Video2", gray)

    width = int(frame1.shape[1]) # Creo que quedo de otro codigo (Revisar)
    height = int(frame1.shape[0])

    faces_rect = haar_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=3)

    for (x,y,w,h) in faces_rect:
        x_value = int(w / 5)
        y_value = int(h / 5)
        x_val = int(x + (w/2))
        y_val = int(y + (h/2))

        cv.rectangle(frame2, (x,y), (x+w,y+h), (0,255,0), thickness=2)
        
        # Esto se puede sacar del for
        laser(frame2)

        if (laser_valueX > x+x_value and laser_valueX < x+w-x_value and laser_valueY > y+y_value and laser_valueY < y+h-y_value): # Dentro del cuadrado chico
            cv.rectangle(frame2, (x+x_value,y+y_value), (x+w-x_value,y+h-y_value), (0,255,255), thickness=2)
            send_serialX = "S"
            send_serialY = "S"
        else:
            cv.rectangle(frame2, (x+x_value,y+y_value), (x+w-x_value,y+h-y_value), (0,255,0), thickness=2)

            if (laser_valueX < x+x_value):
                send_serialX = "L"
            elif (laser_valueX > x+w-x_value):
                send_serialX = "R"
            elif (laser_valueX > x+x_value and laser_valueX < x+w-x_value):
                send_serialX = "S"

            if (laser_valueY < y+y_value):
                send_serialY = "D"
            elif (laser_valueY > y+h-y_value):
                send_serialY = "U"
            elif (laser_valueY > y+y_value and laser_valueY < y+h-y_value):
                send_serialY = "S"

        if(send_serialX != actual_send_serialX or send_serialY != actual_send_serialY): # Enviar solo cuando cambia
            print(send_serialX, send_serialY)

            actual_send_serialX = send_serialX
            actual_send_serialY = send_serialY
            # Falta enviar
  

    numpy_horizontal = np.hstack((frame2, frame1)) # Para juntar las ventanas
    cv.imshow('FaceDetection', numpy_horizontal)

    if cv.waitKey(19) & 0xFF==ord("f"):  # Tecla "f" para romper el while
        break

capture.release()
cv.destroyAllWindows()
