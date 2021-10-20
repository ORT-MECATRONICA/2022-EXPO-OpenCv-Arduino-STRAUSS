import cv2 as cv
import serial
import numpy as np
import time

PORT = input("Ingrese el puerto serie: ")
# PORT = "COM4"

camera_angleX = int(input("Ingrese el el angulo de la camara (x): "))
camera_angleY = int(input("Ingrese el el angulo de la camara (y): "))
# camera_angleX = 36 # Cambiar
# camera_angleY = 30

desvioX = int(input("Ingrese el desvio en x: "))
desvioY = int(input("Ingrese el desvio en y: "))
# desvioX = 62 # Cambiar
# desvioY = 58

# Solo para azul
azulBajo = np.array([90, 100, 20], np.uint8)
azulAlto = np.array([120, 255, 255], np.uint8)

AREA = int(input("Ingrese el area: "))
# AREA = 2000

DELAY = float(input("Ingrese delay: "))

VIDEO = int(input("Video: "))
capture = cv.VideoCapture(VIDEO, cv.CAP_DSHOW) # 0 para la camara default
serialArduino = serial.Serial(PORT, 115200)


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
    frameHSV = cv.cvtColor(frame2, cv.COLOR_BGR2HSV)
    
    mask = cv.inRange(frameHSV, azulBajo, azulAlto)
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cv.drawContours(frame2, contours, -1, (0,255,0), 4)

    width = int(frame1.shape[1]) # Tamaño de la imagen
    height = int(frame1.shape[0])

    for c in contours:
        area = cv.contourArea(c)

        if area > AREA:
            M = cv.moments(c)

            if M['m00'] == 0:
                M['m00'] = 1

            x = int(M['m10'] / M['m00'])
            y = int(M['m01'] / M['m00'])

            cv.circle(frame2, (x, y), 5 , (0,0,255), thickness=-1)
            newContour = cv.convexHull(c)
            cv.drawContours(frame2, [newContour], 0, (0,255,0), 3)

            send_x = int((x / width * camera_angleX) + desvioX) # Paso de pixeles a angulos y sumo el desvio de angulos entre servos y camara
            send_y = int((y / height * camera_angleY) + desvioY)
            
            sendSerial = str(send_x) + "," + str(send_y)
            print(sendSerial)
            serialArduino.write(sendSerial.encode('ascii'))
            time.sleep(DELAY)

    numpy_horizontal = np.hstack((frame2, frame1)) # Para juntar las ventanas
    cv.imshow('FaceDetection', numpy_horizontal)

    if cv.waitKey(19) & 0xFF==ord("f"):  # Tecla "f" para romper el while
        break

capture.release()
cv.destroyAllWindows()
