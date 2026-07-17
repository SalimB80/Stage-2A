#!/usr/bin/env python3
"""
tidy_dataset.py — range un dossier dataset en sous-dossiers autonomes PAR SESSION.

Apres `collect`, tout atterrit a plat dans dataset_collected/tortugaX/. Ce script
range chaque session dans son propre dossier, autonome et supprimable d'un bloc :

  tortuga2/
    tortuga2_20260716_190132/                 <- une session (supprimable d'un coup)
        tortuga2_20260716_190132_final.mp4    #  video complete
        frames_total.csv                      #  frames.csv de tous les segments, fusionnes
        odom_total.csv                        #  odometrie de tous les segments
        scan_total.csv                        #  lidar de tous les segments
        raw/                                  #  tout le brut
            tortuga2_20260716_190132_seg01/   #    images + frames/odom/scan.csv
            tortuga2_20260716_190132_seg01.mp4
            ...

Les *_total.csv concatenent les CSV des segments (dans l'ordre seg01..segNN) avec
une colonne `segment` en tete pour retrouver le brut. Une session ratee = un seul
dossier a supprimer.

  ./tidy_dataset.py <dossier> [--dry-run] [--tol SECONDES]

- <dossier> : un dossier robot (dataset_collected/tortuga2) OU le parent
              (dataset_collected) -> chaque tortugaX/ est traite.
- --dry-run : montre ce qui SERAIT fait, sans rien deplacer ni ecrire.
- --tol N   : tolerance (s) pour ranger un ancien rosbag pres de sa session
              (defaut 300). Les rosbags vont dans raw/ (plus enregistres).

Sur : deplace seulement (jamais de suppression), saute les destinations
existantes, idempotent (relancable sans risque).
"""

import csv
import os
import re
import shutil
import sys
from datetime import datetime

TS = r'(\d{8}_\d{6})'
ROBOT = r'(tortuga\d+)'

RE_ROBOT_DIR = re.compile(r'^tortuga\d+$')
RE_SESSION_DIR = re.compile(rf'^{ROBOT}_{TS}$')            # conteneur de session
RE_SEG = re.compile(rf'^{ROBOT}_{TS}_seg\d+(?:\..+)?$')    # dossier OU fichier
RE_FINAL = re.compile(rf'^{ROBOT}_{TS}_final\.mp4$')
RE_BAGDIR = re.compile(rf'^bag_{TS}_{ROBOT}$')
RE_BAGFILE = re.compile(rf'^bag_{TS}_{ROBOT}_.+$')

TOTALS = ('frames', 'odom', 'scan')                       # <kind>.csv -> <kind>_total.csv


def _parse(ts):
    return datetime.strptime(ts, '%Y%m%d_%H%M%S')


def _match_session(bag_ts, sessions, tol_s):
    if not sessions:
        return bag_ts
    bt = _parse(bag_ts)
    best = min(sessions, key=lambda s: abs((_parse(s) - bt).total_seconds()))
    if abs((_parse(best) - bt).total_seconds()) <= tol_s:
        return best
    return bag_ts


def _sdir(robot_dir, robot, sess):
    d = os.path.join(robot_dir, f'{robot}_{sess}')
    return d, os.path.join(d, 'raw')


def _read_csv_rows(path):
    """Lit un CSV en TOLERANT les fichiers abimes : octets NUL (recorder tue en
    pleine ecriture) retires, decodage permissif. Renvoie (header, rows) ; header
    None si vide. Une ligne corrompue arrete la lecture de CE fichier sans casser
    le reste."""
    with open(path, newline='', encoding='utf-8', errors='replace') as fh:
        rd = csv.reader(line.replace('\x00', '') for line in fh)
        header = next(rd, None)
        if header is None:
            return None, []
        rows = []
        try:
            for r in rd:
                if r:
                    rows.append(r)
        except csv.Error as e:
            print(f"  ! {os.path.basename(os.path.dirname(path))}/"
                  f"{os.path.basename(path)} : lignes ignorees ({e})")
        return header, rows


def build_totals(session_dir):
    """Concatene raw/*_segNN/<kind>.csv -> <kind>_total.csv (ajoute une colonne
    `segment`). Renvoie la liste des totaux ecrits ; previent si un flux est vide
    (ex. odom.csv a 0 octet = pas d'odometrie enregistree)."""
    raw = os.path.join(session_dir, 'raw')
    if not os.path.isdir(raw):
        return []
    segdirs = sorted(d for d in os.listdir(raw)
                     if RE_SEG.match(d) and os.path.isdir(os.path.join(raw, d)))
    written = []
    for kind in TOTALS:
        header, rows = None, []
        for seg in segdirs:
            f = os.path.join(raw, seg, f'{kind}.csv')
            if not os.path.isfile(f):
                continue
            tag = seg.rsplit('_', 1)[-1]                  # 'seg01'
            h, rs = _read_csv_rows(f)
            if h is None:
                continue
            if header is None:
                header = ['segment'] + h
            rows.extend([tag] + r for r in rs)
        if header and rows:
            out = os.path.join(session_dir, f'{kind}_total.csv')
            with open(out, 'w', newline='') as fh:
                w = csv.writer(fh)
                w.writerow(header)
                w.writerows(rows)
            written.append((os.path.basename(out), len(rows)))
        elif segdirs:
            print(f"  ! {os.path.basename(session_dir)} : {kind}_total.csv VIDE "
                  f"(aucune donnee {kind} dans les segments)")
    return written


def tidy(robot_dir, dry_run=False, tol_s=300):
    robot_dir = os.path.abspath(robot_dir.rstrip('/'))
    if not os.path.isdir(robot_dir):
        print(f"[tidy] pas un dossier : {robot_dir}")
        return
    top = sorted(os.listdir(robot_dir))

    # Sessions connues : depuis les segments/finaux ET les conteneurs existants.
    sessions = {}
    for name in top:
        m = RE_SEG.match(name) or RE_FINAL.match(name) or RE_SESSION_DIR.match(name)
        if m:
            sessions[m.group(2)] = m.group(1)

    ops = []            # (src, dest)
    touched = {}        # (robot, sess) -> session_dir

    def place(name, src, robot, sess, kind):
        sd, rd = _sdir(robot_dir, robot, sess)
        dest = os.path.join(sd if kind == 'final' else rd, name)
        if os.path.abspath(src) != os.path.abspath(dest):
            ops.append((src, dest))
        touched[(robot, sess)] = sd

    # 1) Elements a plat au niveau du robot.
    for name in top:
        if RE_SESSION_DIR.match(name):
            continue
        src = os.path.join(robot_dir, name)
        mf, ms = RE_FINAL.match(name), RE_SEG.match(name)
        mb = RE_BAGDIR.match(name) or RE_BAGFILE.match(name)
        if mf:
            place(name, src, mf.group(1), mf.group(2), 'final')
        elif ms:
            place(name, src, ms.group(1), ms.group(2), 'raw')
        elif mb:
            bag_ts, robot = mb.group(1), mb.group(2)
            sess = _match_session(bag_ts, sessions, tol_s)
            sessions.setdefault(sess, robot)
            place(name, src, robot, sess, 'raw')

    # 2) Conteneurs de session deja existants : on normalise leur contenu
    #    (tout ce qui n'est pas le _final.mp4 ni raw/ part dans raw/).
    for name in top:
        md = RE_SESSION_DIR.match(name)
        if not md:
            continue
        robot, sess = md.group(1), md.group(2)
        sd, rd = _sdir(robot_dir, robot, sess)
        touched[(robot, sess)] = sd
        for inner in sorted(os.listdir(sd)):
            if inner == 'raw' or RE_FINAL.match(inner):
                continue
            if inner in (f'{k}_total.csv' for k in TOTALS):
                continue
            isrc = os.path.join(sd, inner)
            dest = os.path.join(rd, inner)
            if os.path.abspath(isrc) != os.path.abspath(dest):
                ops.append((isrc, dest))

    # 3) Execution des deplacements.
    moved = skipped = 0
    for src, dest in ops:
        rel = os.path.relpath(dest, robot_dir)
        if os.path.exists(dest):
            print(f"  ! existe deja, saute : {rel}")
            skipped += 1
            continue
        if dry_run:
            print(f"  [dry-run] {os.path.basename(src)}  ->  {rel}")
            moved += 1
            continue
        os.makedirs(os.path.dirname(dest), exist_ok=True)
        shutil.move(src, dest)
        print(f"  {os.path.basename(src)}  ->  {rel}")
        moved += 1

    # 4) (Re)construction des *_total.csv pour chaque session touchee.
    if dry_run:
        for (robot, sess) in sorted(touched):
            print(f"  [dry-run] {robot}_{sess} : construirait frames/odom/scan_total.csv")
    else:
        for (robot, sess), sd in sorted(touched.items()):
            for fname, n in build_totals(sd):
                print(f"  {os.path.relpath(os.path.join(sd, fname), robot_dir)}"
                      f"  ({n} lignes)")

    tag = "(dry-run) " if dry_run else ""
    if moved or skipped or touched:
        print(f"[tidy] {os.path.basename(robot_dir)} : {tag}{moved} deplace(s)"
              + (f", {skipped} saute(s)" if skipped else "")
              + f", {len(touched)} session(s)")
    else:
        print(f"[tidy] {os.path.basename(robot_dir)} : rien a ranger.")


def run(path, dry_run=False, tol_s=300):
    path = path.rstrip('/')
    if not os.path.isdir(path):
        print(f"[tidy] introuvable : {path}")
        return 1
    subs = [d for d in sorted(os.listdir(path))
            if RE_ROBOT_DIR.match(d) and os.path.isdir(os.path.join(path, d))]
    if subs:
        for d in subs:
            tidy(os.path.join(path, d), dry_run, tol_s)
    else:
        tidy(path, dry_run, tol_s)
    return 0


def main(argv):
    args = [a for a in argv[1:] if not a.startswith('-')]
    dry = '--dry-run' in argv or '-n' in argv
    tol = 300
    if '--tol' in argv:
        try:
            tol = float(argv[argv.index('--tol') + 1])
        except (ValueError, IndexError):
            pass
    if not args:
        print(__doc__)
        return 1
    return run(args[0], dry, tol)


if __name__ == '__main__':
    sys.exit(main(sys.argv))
