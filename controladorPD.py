import WebGUI
import HAL
import Frequency
import cv2


KP = 0.010
KD = 0.011 

IMG_CENTER = 320

V_MAX = 28.00
V_MIN = 8.00

prev_err = 0

while True:
    img = HAL.getImage()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    red_mask = cv2.inRange(hsv, (0, 125, 125), (30,255,255))
    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)

        if M["m00"] != 0:
            cX = M["m10"] / M["m00"]
            err = IMG_CENTER - cX

            # Término derivativo
            derivative = err - prev_err

            # Control PD
            w_dyn = KP * err + KD * derivative

            v_dyn = V_MAX - ((abs(err) / IMG_CENTER) * (V_MAX - V_MIN))

            HAL.setV(v_dyn)
            HAL.setW(w_dyn)

            # Actualizar error anterior
            prev_err = err 
        else:
            HAL.setV(0)
            HAL.setW(0)
    else:
        # Línea perdida: girar hacia donde se vio por última vez
        HAL.setV(0.5)
        if prev_err > 0:
            HAL.setW(1.5)
        else:
            HAL.setW(-1.5)
        
    WebGUI.showImage(img)