import cv2 as cv
import serial
import numpy as np
import time
import math

mitadX = 0
valorCalibracion1 = 180
valorCalibracion2 = 0

capture = cv.VideoCapture(0)
haar_cascade = cv.CascadeClassifier("haar_face.xml")
# serialArduino = serial.Serial("COM7",9600) #Descomentar cuando haya algo en el puerto

def rescaleFrame(frame, scale=0.75):  # Rescalar el video (Default = 0.75) 
    width = int(frame.shape[1]  * scale)
    height = int(frame.shape[0] * scale)

    dimensions = (width,height)

    return cv.resize(frame, dimensions, interpolation=cv.INTER_AREA) # Que es el INTER_AREA ???

valorCalibracion1 = int(input("Introduzca el valor de calibracion MAXIMO del eje X: "))
valorCalibracion2 = int(input("Introduzca el valor de calibracion MINIMO del eje X: "))
valorCalibracionResta = valorCalibracion1 - valorCalibracion2

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
        cv.rectangle(frame2, (x,y), (x+w,y+h), (0,255,0), thickness=2)
        mitadX = (x + math.floor(w/2)) #la mitad del cuadrado/rec

    numpy_horizontal = np.hstack((frame2, frame1))
    cv.imshow('FaceDetection', numpy_horizontal)

    #envioSerial = math.floor((mitadX / width) * valorCalibracionResta + valorCalibracion2)
    #serialArduino.write(str(envioSerial).encode('ascii'))
    #time.sleep(1)

    #print(envioSerial)

    if cv.waitKey(19) & 0xFF==ord("f"):  # Tecla "d" para romper el while
        break

capture.release()
cv.destroyAllWindows()
