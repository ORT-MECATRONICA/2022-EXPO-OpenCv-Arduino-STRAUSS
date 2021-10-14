import cv2 as cv
import serial
import numpy as np
import time

camera_angleX = int(input("Ingrese el el angulo de la camara (x): "))
camera_angleY = int(input("Ingrese el el angulo de la camara (y): "))
# camera_angleX = 36 # Cambiar
# camera_angleY = 30

desvioX = int(input("Ingrese el desvio en x: "))
desvioY = int(input("Ingrese el desvio en y: "))
# desvioX = 62 # Cambiar
# desvioY = 58

PORT = input("Ingrese el puerto serie: ")
# PORT = "COM4"

capture = cv.VideoCapture(0, cv.CAP_DSHOW) # 0 para la camara default
haar_cascade = cv.CascadeClassifier("Python/haar_faceee.xml") # pPner ubicacion del archivo (Me genero problemas)
# Este archivo es para la deteccion de caras.
serialArduino = serial.Serial(PORT,9600)


def rescaleFrame(frame, scale=0.75):  # Rescalar el video (Default = 0.75) 
    width = int(frame.shape[1]  * scale)
    height = int(frame.shape[0] * scale)

    dimensions = (width,height)

    return cv.resize(frame, dimensions, interpolation=cv.INTER_AREA)


while True:
    isTrue, frame1 = capture.read(0)
    frame1 = cv.flip(frame1, 1)
    frame1 = rescaleFrame(frame1, 1) # Cambiar el valor  para aumentar o achicar la imagen
    frame2 = rescaleFrame(frame1, 1)
    gray = cv.cvtColor(frame2, cv.COLOR_BGR2GRAY)
    #cv.imshow("Video2", gray)

    width = int(frame1.shape[1]) # Tamaño de la imagen
    height = int(frame1.shape[0])
    # print(width, height) 

    faces_rect = haar_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=3)

    for (x,y,w,h) in faces_rect:
        x_value = int(w / 5)
        y_value = int(h / 5)
        x_val = int(x + (w/2))
        y_val = int(y + (h/2))

        cv.rectangle(frame2, (x,y), (x+w,y+h), (255,0,0), thickness=2)
        cv.circle(frame2, (x_val, y_val), 5 , (0,0,255), thickness=-1)

        send_x = int((x_val / width * camera_angleX) + desvioX) # Paso de pixeles a angulos y sumo el desvio de angulos entre servos y camara
        send_y = int((y_val / height * camera_angleY) + desvioY)

        sendSerial = str(send_x) + "," + str(send_y)
        print(sendSerial)

        serialArduino.write(sendSerial.encode('ascii'))
        time.sleep(1)

        
    numpy_horizontal = np.hstack((frame2, frame1)) # Para juntar las ventanas
    cv.imshow('FaceDetection', numpy_horizontal)

    if cv.waitKey(19) & 0xFF==ord("f"):  # Tecla "f" para romper el while
        break

capture.release()
cv.destroyAllWindows()
