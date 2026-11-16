import WebGUI
import HAL

import cv2

KP = 0.01       
V = 8.0         

i = 0
last_err = 0
err = 0
w = 0
while True:
    img = HAL.getImage()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    red_mask = cv2.inRange(hsv,
                           ( 0, 125, 125),
                           (30, 255, 255))
    contours, hierarchy = cv2.findContours(red_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)

    cX, cY = 0, 0
    if len(contours) > 0:
        M = cv2.moments(max(contours, key=cv2.contourArea))
        if M["m00"] != 0:
            cX = M["m10"] / M["m00"]
            cY = M["m01"] / M["m00"]

    if cX > 0:
        err = 320 - cX   
        last_err = err
        w = KP * err
        HAL.setV(V)
        HAL.setW(w)
    else:
        # Línea perdida: girar hacia donde se vio por última vez
        HAL.setV(0.5)
        if last_err > 0:
            HAL.setW(1.5)
        else:
            HAL.setW(-1.5)

    WebGUI.showImage(red_mask)
    print('%d cX: %.2f err: %.2f w: %.2f' % (i, cX, err, w))
    i = i + 1
