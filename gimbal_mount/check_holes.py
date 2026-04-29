"""Check all 8 M2.5 hole positions exist in motor_back_mount.stl"""
import trimesh
import numpy as np
import math

p = r'c:\Users\Administrator\Desktop\chendian\gimbal_mount\motor_back_mount.stl'
m = trimesh.load(p, force='mesh')

PLATE_T  = 6.0
PLATE_H  = 68.0
PCD_R    = 12.5
M25_R    = 1.5
CENTER_R = 9.0

print('=== Hole Position Verification ===\n')
print(f'Center hole: r={CENTER_R}mm (phi{CENTER_R*2}mm)')
print(f'M2.5 holes : r={M25_R}mm at PCD_R={PCD_R}mm')
print(f'Wall gap   : {PCD_R - CENTER_R - M25_R:.1f}mm\n')

# For each of the 8 expected M2.5 holes, shoot a ray through
# the plate at the expected hole center position
# If it passes through (no hit), the hole exists
# If it hits the plate, the hole is missing

Y_TEST = PLATE_T / 2  # middle of plate thickness

found = 0
missing = 0
for ang_deg in range(0, 360, 45):
    ang = math.radians(ang_deg)
    cx = PCD_R * math.cos(ang)
    cz = PCD_R * math.sin(ang) + PLATE_H / 2

    # Cast ray in Y direction through the expected hole center
    origin = np.array([[cx, -1.0, cz]])
    direction = np.array([[0.0, 1.0, 0.0]])
    
    hits = m.ray.intersects_location(origin, direction)
    hit_points = hits[0]
    
    if len(hit_points) == 0:
        # No intersection = ray passes through empty space = hole exists
        status = 'HOLE OK'
        found += 1
    elif len(hit_points) >= 2:
        # Two hits = enters and exits solid = hole is filled (no hole)
        status = 'MISSING - solid here!'
        missing += 1
    else:
        status = f'EDGE ({len(hit_points)} hits)'
        found += 1

    pattern = 'BACK' if ang_deg % 90 == 0 else 'FRONT'
    print(f'  {ang_deg:3d} deg  X={cx:+6.1f} Z={cz:5.1f}  [{pattern}]  {status}')

print(f'\nSummary: {found} holes found, {missing} missing')
print(f'Back-face pattern (0/90/180/270): need all 4')
print(f'Front-face pattern (45/135/225/315): need all 4')
