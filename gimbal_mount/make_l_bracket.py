"""
GM3506 Motor Back-Mount L-Bracket Generator
=============================================
Replaces the failed U-clamp design.
The motor's BACK FACE (stator side) has 4x M2.5 threaded holes.
This L-bracket bolts directly to those holes.

Assembly:
  - Vertical plate  → bolts to motor back face with 4x M2.5x6
  - Horizontal foot → bolts to table/board with 4x M4
  - Motor axis is horizontal, arm extends horizontally, stabilization works!
"""
import sys, subprocess, os, math

for pkg in ['manifold3d', 'trimesh', 'numpy']:
    try: __import__(pkg)
    except ImportError:
        subprocess.check_call([sys.executable, '-m', 'pip', 'install', '--quiet', pkg])

from manifold3d import Manifold
import trimesh, numpy as np

OUT = os.path.dirname(os.path.abspath(__file__))

def cyl(r, h, seg=48):
    return Manifold.cylinder(h, r, r, seg)

def box(x, y, z):
    return Manifold.cube([x, y, z])

def mv(m, dx=0., dy=0., dz=0.):
    return m.translate([dx, dy, dz])

def m_to_trimesh(m: Manifold) -> trimesh.Trimesh:
    mesh = m.to_mesh()
    v = np.array(mesh.vert_properties)
    f = np.array(mesh.tri_verts).reshape(-1, 3)
    return trimesh.Trimesh(vertices=v, faces=f)


# ═══════════════════════════════════════════════════════════════════════════
# Motor Back-Mount L-Bracket
# ═══════════════════════════════════════════════════════════════════════════
# Motor back face holes: 4x M2.5 at PCD=25mm
# Making ALL 8 positions (every 45°) so it works regardless of angle orientation
PCD_R  = 12.5  # PCD = 25mm, radius = 12.5mm
M25_R  = 1.5   # M2.5 clearance (phi 3.0mm)
M4_R   = 2.3   # M4 clearance (phi 4.6mm)

PLATE_W  = 68.0   # Vertical plate width (wider than motor OD=40mm)
PLATE_H  = 68.0   # Vertical plate height
PLATE_T  = 6.0    # Plate thickness
FOOT_D   = 50.0   # Horizontal foot depth
FOOT_T   = 6.0    # Foot thickness
CORNER_R = 5.0    # Corner fillet radius (visual quality)

def make_l_bracket() -> Manifold:
    # ── Vertical plate (motor mounting face) ─────────────────────────────
    plate = mv(box(PLATE_W, PLATE_T, PLATE_H), -PLATE_W/2, 0, 0)

    # 8x M2.5 holes at every 45°
    # (back face holes are at 0/90/180/270; front face at 45/135/225/315)
    # Use all 8 to be compatible with both orientations
    for ang_deg in range(0, 360, 45):
        ang = math.radians(ang_deg)
        cx = PCD_R * math.cos(ang)
        cz = PCD_R * math.sin(ang) + PLATE_H / 2  # centered vertically
        hole = cyl(M25_R, PLATE_T + 2, 20)
        hole = hole.rotate([-90, 0, 0]).translate([cx, -1, cz])
        plate = plate - hole

    # Center relief hole (encoder PCB clearance, phi18mm)
    # Must be < PCD_R - M25_R - 2mm wall = 12.5-1.5-2 = 9mm radius
    center_hole = cyl(9.0, PLATE_T + 2, 36)
    center_hole = center_hole.rotate([-90, 0, 0]).translate([0, -1, PLATE_H/2])
    plate = plate - center_hole

    # Cable slot (for encoder wire routing, 10x12mm notch at bottom)
    cable_slot = mv(box(10, PLATE_T + 2, 15), -5, -1, 0)
    plate = plate - cable_slot

    # ── Horizontal foot (desk/table mounting) ────────────────────────────
    foot = mv(box(PLATE_W, FOOT_D, FOOT_T), -PLATE_W/2, 0, -FOOT_T)

    # 4x M4 holes in foot corners
    for xi in [-(PLATE_W/2 - 12), PLATE_W/2 - 12]:
        for yi in [14, FOOT_D - 12]:
            m4 = cyl(M4_R, FOOT_T + 2, 20)
            m4 = mv(m4, xi, yi, -FOOT_T - 1)
            foot = foot - m4

    # M4 countersink cavities (head recess 8mm dia, 2mm deep) for flush bolt head
    for xi in [-(PLATE_W/2 - 12), PLATE_W/2 - 12]:
        for yi in [14, FOOT_D - 12]:
            csk = cyl(4.5, 2.5, 20)
            csk = mv(csk, xi, yi, -FOOT_T - 1)
            foot = foot - csk

    # ── Join plate + foot ─────────────────────────────────────────────────
    bracket = plate + foot

    return bracket


# ═══════════════════════════════════════════════════════════════════════════
if __name__ == '__main__':
    print('=' * 55)
    print('GM3506 L-Bracket Back-Mount Generator v1')
    print('=' * 55)

    print('\nGenerating motor_back_mount.stl ...')
    m = make_l_bracket()
    t = m_to_trimesh(m)
    path = os.path.join(OUT, 'motor_back_mount.stl')
    t.export(path)
    ext = t.extents.round(1)
    vol = abs(t.volume)
    print(f'  Watertight : {t.is_watertight}')
    print(f'  Extents    : {ext[0]} x {ext[1]} x {ext[2]} mm')
    print(f'  Volume     : {vol:.0f} mm3  (~{vol*1.27/1000:.1f} g PETG)')
    print(f'  -> {path}')
    print('\nDone!')
    print()
    print('Assembly:')
    print('  1. Vertical plate face faces the motor BACK face')
    print('  2. 4 of the 8 M2.5 holes align with motor back face - use those 4')
    print('  3. Foot sits flat on desk, bolt with 4x M4x20')
    print('  4. Bracket + motor is now fixed, motor axis is horizontal')
    print('  5. Attach rotor_arm to motor FRONT face (4x M2.5x6)')
    print()
    print('Order summary:')
    print('  Print: motor_back_mount.stl + rotor_arm.stl (PETG 0.2mm)')
    print('  Buy  : 4x M2.5x6 (arm to front face)')
    print('       + 4x M2.5x8 (bracket to back face)')
    print('       + 4x M4x20  (bracket foot to desk/board)')
