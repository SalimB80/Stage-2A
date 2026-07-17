#!/usr/bin/env python3
"""
bag_to_csv.py — convert a ROS 2 bag (.db3 sqlite) into human-readable CSVs.

Le mode dataset enregistre un rosbag `bag_<date>_<robot>` contenant les topics
`.../scan` (LaserScan), `.../odom` (Odometry) et `.../imu` (Imu). Un `.db3` est
une base SQLite illisible telle quelle : ce script la deserialise et ecrit UN
CSV PAR TOPIC, place a cote de la video par defaut.

  ./bag_to_csv.py <bag.db3 | dossier_du_bag> [dossier_de_sortie]

- Accepte un `.db3` isole OU un dossier de bag (plusieurs `*.db3` + metadata).
- Sortie : <dossier_de_sortie>/<nom_du_bag>_<topic>.csv
           (dossier_de_sortie par defaut = le dossier qui contient le .db3)
- Deserialisation :
    * si ROS 2 (rclpy) est source -> N'IMPORTE QUEL type de message (generique).
    * sinon -> decodeur CDR interne pour LaserScan / Odometry / Imu (les topics
      enregistres par ce projet), donc aucune dependance ROS necessaire sur le PC.

Chaque ligne porte `bag_time_ns` (horodatage de reception du bag) et, quand le
message a un header, `stamp_sec` / `stamp_nanosec` — directement alignables avec
frames.csv / scan.csv / odom.csv du recorder (meme horloge ROS).
"""

import csv
import glob
import os
import sqlite3
import struct
import sys


# --------------------------------------------------------------------------
# Decodeur CDR minimal (fallback sans ROS) pour les types de ce projet.
# CDR : 4 octets d'encapsulation en tete (le 2e octet code l'endianness), puis
# les membres alignes sur leur taille RELATIVEMENT au debut du corps (apres ces
# 4 octets). Sequences/strings = prefixe uint32 (longueur).
# --------------------------------------------------------------------------
class _CDR:
    def __init__(self, data):
        self.buf = data
        self.little = len(data) >= 2 and (data[1] & 1) == 1
        self.base = 4          # les membres s'alignent apres l'entete d'encaps.
        self.pos = 4

    def _align(self, size):
        rel = (self.pos - self.base) % size
        if rel:
            self.pos += size - rel

    def _one(self, fmt, size):
        self._align(size)
        v = struct.unpack_from(('<' if self.little else '>') + fmt,
                               self.buf, self.pos)[0]
        self.pos += size
        return v

    def i32(self):
        return self._one('i', 4)

    def u32(self):
        return self._one('I', 4)

    def f32(self):
        return self._one('f', 4)

    def f64(self):
        return self._one('d', 8)

    def string(self):
        n = self.u32()
        raw = self.buf[self.pos:self.pos + n]
        self.pos += n
        return raw.split(b'\x00', 1)[0].decode('utf-8', 'replace')

    def f32_seq(self):
        return [self.f32() for _ in range(self.u32())]

    def f64_n(self, n):
        return [self.f64() for _ in range(n)]


def _header(cdr):
    sec = cdr.i32()
    nsec = cdr.u32()
    frame = cdr.string()
    return {'stamp_sec': sec, 'stamp_nanosec': nsec, 'frame_id': frame}


def _seq(vals):
    return ';'.join(f'{v:.6g}' for v in vals)


def _decode_laserscan(data):
    c = _CDR(data)
    r = _header(c)
    r['angle_min'] = c.f32()
    r['angle_max'] = c.f32()
    r['angle_increment'] = c.f32()
    r['time_increment'] = c.f32()
    r['scan_time'] = c.f32()
    r['range_min'] = c.f32()
    r['range_max'] = c.f32()
    r['ranges'] = _seq(c.f32_seq())
    r['intensities'] = _seq(c.f32_seq())
    return r


def _decode_odometry(data):
    c = _CDR(data)
    r = _header(c)
    r['child_frame_id'] = c.string()
    px, py, pz = c.f64(), c.f64(), c.f64()
    qx, qy, qz, qw = c.f64(), c.f64(), c.f64(), c.f64()
    c.f64_n(36)                                     # pose covariance (ignoree)
    lx, ly, lz = c.f64(), c.f64(), c.f64()
    ax, ay, az = c.f64(), c.f64(), c.f64()
    c.f64_n(36)                                     # twist covariance (ignoree)
    import math
    yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                     1.0 - 2.0 * (qy * qy + qz * qz))
    r.update({'x': px, 'y': py, 'z': pz,
              'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw, 'yaw': yaw,
              'vx': lx, 'vy': ly, 'vz': lz,
              'wx': ax, 'wy': ay, 'wz': az})
    return r


def _decode_imu(data):
    c = _CDR(data)
    r = _header(c)
    ox, oy, oz, ow = c.f64(), c.f64(), c.f64(), c.f64()
    c.f64_n(9)
    gx, gy, gz = c.f64(), c.f64(), c.f64()
    c.f64_n(9)
    ax, ay, az = c.f64(), c.f64(), c.f64()
    c.f64_n(9)
    r.update({'ori_x': ox, 'ori_y': oy, 'ori_z': oz, 'ori_w': ow,
              'gyro_x': gx, 'gyro_y': gy, 'gyro_z': gz,
              'acc_x': ax, 'acc_y': ay, 'acc_z': az})
    return r


_FALLBACK = {
    'sensor_msgs/msg/LaserScan': _decode_laserscan,
    'nav_msgs/msg/Odometry': _decode_odometry,
    'sensor_msgs/msg/Imu': _decode_imu,
}


# --------------------------------------------------------------------------
# Chemin generique via rclpy (n'importe quel type), si ROS 2 est source.
# --------------------------------------------------------------------------
def _try_ros_deserializer(type_name):
    try:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except Exception:
        return None
    try:
        msg_cls = get_message(type_name)
    except Exception:
        return None

    def flatten(obj, prefix=''):
        out = {}
        fields = getattr(obj, 'get_fields_and_field_types', None)
        if fields is None:                          # feuille (primitif)
            return {prefix.rstrip('.'): obj}
        for name in fields():
            val = getattr(obj, name)
            key = f'{prefix}{name}'
            if isinstance(val, (list, tuple)) or (
                    hasattr(val, '__len__') and not isinstance(val, str)
                    and not hasattr(val, 'get_fields_and_field_types')):
                try:
                    out[key] = _seq([float(v) for v in val])
                except (TypeError, ValueError):
                    out[key] = ';'.join(str(v) for v in val)
            elif hasattr(val, 'get_fields_and_field_types'):
                out.update(flatten(val, key + '.'))
            else:
                out[key] = val
        return out

    def deser(data):
        msg = deserialize_message(bytes(data), msg_cls)
        row = flatten(msg)
        # remonte le header a plat pour l'alignement temporel
        if 'header.stamp.sec' in row:
            row['stamp_sec'] = row.pop('header.stamp.sec')
            row['stamp_nanosec'] = row.pop('header.stamp.nanosec')
            row['frame_id'] = row.pop('header.frame_id', '')
        return row

    return deser


# --------------------------------------------------------------------------
def _resolve_db3(path):
    """Renvoie (liste_de_.db3, nom_de_base_du_bag)."""
    if os.path.isdir(path):
        db3s = sorted(glob.glob(os.path.join(path, '*.db3')))
        base = os.path.basename(path.rstrip('/'))
        return db3s, base
    base = os.path.basename(path)
    if base.endswith('.db3'):
        base = base[:-4]
    # bag_..._tortuga1_0.db3 -> retire l'index de split final "_0"
    parts = base.rsplit('_', 1)
    if len(parts) == 2 and parts[1].isdigit():
        base = parts[0]
    return [path], base


def convert(path, out_dir=None):
    db3s, base = _resolve_db3(path)
    if not db3s:
        print(f"[bag_to_csv] aucun .db3 dans {path}")
        return 1
    if out_dir is None:
        out_dir = os.path.dirname(os.path.abspath(db3s[0]))
    os.makedirs(out_dir, exist_ok=True)

    rows_by_topic = {}       # topic_name -> list[dict]
    type_by_topic = {}
    decoders = {}            # type_name -> callable | None
    skipped = set()

    for db3 in db3s:
        con = sqlite3.connect(f'file:{db3}?mode=ro', uri=True)
        try:
            topics = {tid: (name, typ) for tid, name, typ in con.execute(
                'SELECT id, name, type FROM topics')}
            for topic_id, ts, data in con.execute(
                    'SELECT topic_id, timestamp, data FROM messages '
                    'ORDER BY timestamp'):
                name, typ = topics.get(topic_id, (None, None))
                if name is None:
                    continue
                type_by_topic[name] = typ
                if typ not in decoders:
                    decoders[typ] = (_try_ros_deserializer(typ)
                                     or _FALLBACK.get(typ))
                dec = decoders[typ]
                if dec is None:
                    skipped.add(typ)
                    continue
                try:
                    row = dec(data)
                except Exception as e:            # message corrompu -> on saute
                    print(f"[bag_to_csv] {name}: message ignore ({e})")
                    continue
                row = {'bag_time_ns': ts, **row}
                rows_by_topic.setdefault(name, []).append(row)
        finally:
            con.close()

    if not rows_by_topic:
        print(f"[bag_to_csv] rien a convertir dans {base} "
              f"(types non supportes : {', '.join(sorted(skipped)) or '?'})")
        return 1

    # Nom de fichier : nom COURT du topic (dernier segment) tant qu'il est
    # unique ; sinon on retombe sur le chemin complet (generique, sans collision).
    leaves = {}
    for name in rows_by_topic:
        leaves.setdefault(name.rstrip('/').rsplit('/', 1)[-1], []).append(name)

    written = []
    for name, rows in rows_by_topic.items():
        cols = []
        for r in rows:
            for k in r:
                if k not in cols:
                    cols.append(k)
        leaf = name.rstrip('/').rsplit('/', 1)[-1]
        safe = (leaf if len(leaves[leaf]) == 1
                else name.strip('/').replace('/', '_'))
        out = os.path.join(out_dir, f'{base}_{safe}.csv')
        with open(out, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            w.writerows(rows)
        written.append((out, name, type_by_topic[name], len(rows)))

    print(f"[bag_to_csv] {base} -> {out_dir}")
    for out, name, typ, n in written:
        print(f"  {os.path.basename(out)}  ({name}, {typ}, {n} msg)")
    if skipped:
        print(f"  (types sautes, ROS non source : {', '.join(sorted(skipped))})")
    return 0


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 1
    out_dir = argv[2] if len(argv) > 2 else None
    return convert(argv[1], out_dir)


if __name__ == '__main__':
    sys.exit(main(sys.argv))
