import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cv2.namedWindow('cal')
for n, v in [('Hl', 85), ('Hh', 100), ('Sl', 90),
             ('Sh', 255), ('Vl', 90), ('Vh', 255)]:
    cv2.createTrackbar(n, 'cal', v, 255, lambda x: None)

while True:
    ok, f = cap.read()
    if not ok:
        break
    hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)
    g = lambda n: cv2.getTrackbarPos(n, 'cal')
    low = np.array([g('Hl'), g('Sl'), g('Vl')])
    high = np.array([g('Hh'), g('Sh'), g('Vh')])
    mask = cv2.inRange(hsv, low, high)
    out = cv2.bitwise_and(f, f, mask=mask)
    cv2.imshow('cal', np.hstack([f, out]))
    if cv2.waitKey(1) == 27:
        break
cap.release()
cv2.destroyAllWindows()
