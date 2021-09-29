import sys
import numpy as np
import cv2

vid = cv2.VideoCapture(0)

while(vid.isOpened()):
    success, frame = vid.read()
        
    lower_red = np.array([0,0,255]) 
    upper_red = np.array([220,220,255]) 

    kernel = np.ones((3,3),np.uint8) # no se para que es

    mask_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(mask_HSV, lower_red, upper_red) 
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    #mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    output_mask = cv2.bitwise_and(frame, frame, mask = mask)

    (minVal, maxVal,minLoc, maxLoc) = cv2.minMaxLoc(mask) 
    cv2.circle(frame, maxLoc, 20, (0, 0, 255), 2, cv2.LINE_AA)

    output_hough = cv2.cvtColor(output_mask, cv2.COLOR_BGR2GRAY) 
    output_hough = cv2.medianBlur(output_hough, 5)
    circles = cv2.HoughCircles(output_hough, cv2.HOUGH_GRADIENT, 1, 20, param1=18, param2=8, minRadius=0, maxRadius=15)
 
    #Circulo verde
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            cv2.circle(frame,(i[0],i[1]),i[2],(0,255,0),2)
            cv2.circle(frame,(i[0],i[1]),2,(0,255,0),3)
            aux = "(" + str(i[0]) + ", " + str(i[0]) + ")"
            print (aux)

            
    #Exibe as três janelas
    nomeMask = "HSV Mask: inRange, MORPH_OPEN, MORPH_CLOSE e bitwise_and"
    nomeRGB = "RGB Frame: Cicle e HoughCircules desenhados"
    nomeHough = "GRAY Transformada de Hough: medianBlur"
    

    cv2.namedWindow(nomeMask, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(nomeMask, 800 , 600)
    cv2.imshow(nomeMask, output_mask)
    
    cv2.namedWindow(nomeRGB, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(nomeRGB, 800 , 600)
    cv2.imshow(nomeRGB, frame)

    cv2.namedWindow(nomeHough, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(nomeHough, 800 , 600)
    cv2.imshow(nomeHough, output_hough)

    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

vid.release()
cv2.destroyAllWindows()