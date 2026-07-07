#!/usr/bin/env python3
"""
hsv_tuner.py v2 — calibration HSV sur la camera du robot, par le reseau.

v2 : s'abonne au flux COMPRESSE (JPEG, leger) en priorite, avec QoS capteur.
Le flux brut (14 Mo/s) ne passe pas en Wi-Fi -> ecran noir. Le compresse
passe sans souci. Repli automatique sur le brut si le compresse n'existe pas.

Usage : python3 hsv_tuner.py tortuga3
Touches : P = imprimer seuils | 1/2/3 = presets jaune/cyan/rouge | ECHAP = quitter
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

PRESETS = {
    ord('1'): (20, 35, 80, 255, 80, 255),
    ord('2'): (85, 100, 80, 255, 80, 255),
    ord('3'): (0, 8, 100, 255, 80, 255),
}


class HsvTuner(Node):
    def __init__(self, base):
        super().__init__('hsv_tuner')
        self.bridge = CvBridge()
        self.frame = None
        self.source = None
        # Compresse (prioritaire, leger) ET brut (repli) — QoS capteur
        self.create_subscription(
            CompressedImage, base + '/compressed', self.cb_comp,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, base, self.cb_raw, qos_profile_sensor_data)
        self.get_logger().info(f"Ecoute {base} (+/compressed), QoS capteur")

    def cb_comp(self, msg):
        arr = np.frombuffer(msg.data, np.uint8)
        f = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if f is not None:
            self.frame = f
            if self.source != 'compressed':
                self.source = 'compressed'
                self.get_logger().info("Flux COMPRESSE recu (ideal)")

    def cb_raw(self, msg):
        if self.source == 'compressed':
            return          # le compresse arrive, on ignore le brut
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            if self.source != 'raw':
                self.source = 'raw'
                self.get_logger().info("Flux BRUT recu (lourd en Wi-Fi)")
        except Exception as e:
            self.get_logger().warn(f"CvBridge: {e}")


def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else 'tortuga2'
    base = arg if arg.startswith('/') else f'/{arg}/camera/image_raw'

    rclpy.init()
    node = HsvTuner(base)

    win = 'HSV Tuner'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    for name, val, mx in (('H low', 20, 179), ('H high', 35, 179),
                          ('S low', 80, 255), ('S high', 255, 255),
                          ('V low', 80, 255), ('V high', 255, 255)):
        cv2.createTrackbar(name, win, val, mx, lambda x: None)

    print(f"En attente d'images ({base}[/compressed]) ...")
    print("P = imprimer seuils | 1/2/3 = presets | ECHAP = quitter")

    waiting = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.frame is None:
                waiting += 1
                if waiting % 100 == 0:
                    print("Toujours aucune image... verifie : "
                          "ros2 topic hz " + base)
                if (cv2.waitKey(20) & 0xFF) == 27:
                    break
                continue

            g = lambda n: cv2.getTrackbarPos(n, win)
            low = np.array([g('H low'), g('S low'), g('V low')], np.uint8)
            high = np.array([g('H high'), g('S high'), g('V high')], np.uint8)

            frame = node.frame.copy()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, low, high)
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
            seg = cv2.bitwise_and(frame, frame, mask=mask)

            area = int(cv2.countNonZero(mask))
            label = f"[{node.source}] low={low.tolist()} high={high.tolist()} area={area}"
            cv2.putText(seg, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 255, 255), 1)
            cv2.imshow(win, np.hstack([frame, seg]))

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
            elif key in (ord('p'), ord('P')):
                print(f"  ([{low[0]}, {low[1]}, {low[2]}], "
                      f"[{high[0]}, {high[1]}, {high[2]}]),   # area={area}")
            elif key in PRESETS:
                hl, hh, sl, sh, vl, vh = PRESETS[key]
                for n, v in (('H low', hl), ('H high', hh), ('S low', sl),
                             ('S high', sh), ('V low', vl), ('V high', vh)):
                    cv2.setTrackbarPos(n, win, v)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
