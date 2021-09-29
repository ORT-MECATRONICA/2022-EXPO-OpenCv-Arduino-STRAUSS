import cv2
import numpy as np
cap = cv2.VideoCapture(0)

pts = []
while (1):

    # Take each frame
    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red = np.array([0, 0, 255])
    upper_red = np.array([255, 255, 255])

    # kernel = np.ones((3,3),np.uint8) 

    mask = cv2.inRange(hsv, lower_red, upper_red)
    # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(mask)

    my_red = cv2.countNonZero(mask)
    print(my_red)

    cv2.circle(frame, maxLoc, 20, (0, 0, 255), 2, cv2.LINE_AA)
    cv2.imshow('Track Laser', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()