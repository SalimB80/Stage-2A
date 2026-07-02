import cv2
import numpy as np
import math

# Seuils cyan HSV - A CALIBRER avec tools/calibrate_hsv.py
CYAN_LOW = np.array([12, 120, 60])
CYAN_HIGH = np.array([26, 255, 255])

CAMERA_HFOV = math.radians(62)  # Pi Camera v2 ~62 deg horizontal


def detect_helmet(bgr_image, min_area_frac=0.0015):
    """Retourne (bearing_norm, area_ratio, rectangular) ou None.
    bearing_norm in [-1,1] : -1 = casque a gauche de l'image."""
    hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, CYAN_LOW, CYAN_HIGH)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    h, w = mask.shape
    if area < min_area_frac * h * w:
        return None

    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.04 * peri, True)
    rectangular = 4 <= len(approx) <= 6

    M = cv2.moments(c)
    if M['m00'] == 0:
        return None
    cx = M['m10'] / M['m00']
    bearing_norm = (cx - w / 2.0) / (w / 2.0)
    area_ratio = area / float(h * w)
    return bearing_norm, area_ratio, rectangular


def bearing_to_angle(bearing_norm):
    """Angle reel (rad) dans le repere robot. + = a gauche."""
    # bearing_norm +1 = droite image = -angle robot (REP-103 : +y gauche)
    return -bearing_norm * (CAMERA_HFOV / 2.0)


def range_from_lidar(scan, angle, window=3, max_valid=8.0):
    """Distance lidar (m) dans la direction 'angle' (rad), ou None.
    Gere les lidars 0..2pi : normalise l'angle avant indexation."""
    n = len(scan.ranges)
    if n == 0:
        return None
    # Normalise l'angle dans [angle_min, angle_min + 2pi[
    span = scan.angle_max - scan.angle_min
    a = (angle - scan.angle_min) % (2 * math.pi)
    idx = int(round(a / scan.angle_increment))
    # Fenetre circulaire autour de l'index
    valid = []
    for d in range(-window, window + 1):
        j = (idx + d) % n
        r = scan.ranges[j]
        if scan.range_min < r < max_valid and not math.isinf(r) \
                and not math.isnan(r):
            valid.append(r)
    return min(valid) if valid else None
