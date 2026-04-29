"""
GM3506 Gimbal Mount Generator v4
- Rotor Arm: bolts to rotating output face (M2.5 x4, PCD=25mm)
- Motor Base: simple half-cylinder cradle, zip-tie to hold motor barrel
All manifold3d native API, guaranteed watertight output.
"""
import sys, subprocess, os, math

for pkg in ['manifold3d', 'trimesh', 'numpy']:
    try: __import__(pkg)
    except ImportError:
        subprocess.check_call([sys.executable, '-m', 'pip', 'install', '--quiet', pkg])

from manifold3d import Manifold
import trimesh, numpy as np

OUT = os.path.dirname(os.path.abspath(__file__))

# ─── Helpers ────────────────────────────────────────────────────────────────
def cyl(r, h, seg=64):
    return Manifold.cylinder(h, r, r, seg)

def box(x, y, z):
    return Manifold.cube([x, y, z])

def mv(m, dx=0., dy=0., dz=0.):
    return m.translate([dx, dy, dz])

def rx(m, deg): return m.rotate([deg, 0, 0])
def ry(m, deg): return m.rotate([0, deg, 0])
def rz(m, deg): return m.rotate([0, 0, deg])

def export(m: Manifold, filename: str):
    mesh = m.to_mesh()
    v = np.array(mesh.vert_properties)
    f = np.array(mesh.tri_verts).reshape(-1, 3)
    t = trimesh.Trimesh(vertices=v, faces=f)
    path = os.path.join(OUT, filename)
    t.export(path)
    wt = t.is_watertight
    ext = t.extents.round(1)
    vol = abs(t.volume)
    print(f"  File     : {path}")
    print(f"  Faces    : {len(t.faces)}")
    print(f"  Watertight: {'YES' if wt else '*** NO - FIX NEEDED ***'}")
    print(f"  Extents  : {ext[0]} x {ext[1]} x {ext[2]} mm")
    print(f"  Volume   : {vol:.0f} mm3  (~{vol*1.27/1000:.1f} g PETG)")
    return wt


# ═══════════════════════════════════════════════════════════════════════════
# PART 1: Rotor Arm
# Attaches to the rotating disc face of GM3506 (4x M2.5, PCD=25mm, 45deg)
# ═══════════════════════════════════════════════════════════════════════════
def make_rotor_arm() -> Manifold:
    HUB_R  = 19.0   # hub radius  (motor OD/2=20mm, leave 1mm gap)
    HUB_H  = 5.0    # hub thickness
    PCD_R  = 12.5   # bolt-circle radius (PCD=25mm)
    HOLE_R = 1.45   # M2.5 clearance hole radius (phi 2.9mm)
    CLR_R  = 10.25  # center bore radius  (phi 20.5)
    ARM_OX = HUB_R - 4   # arm overlap with hub
    ARM_L  = 95.0   # arm length beyond hub edge
    ARM_W  = 13.0
    ARM_H  = HUB_H
    TIP_L  = 20.0   # thicker tip for laser clamping
    TIP_W  = 20.0
    TIP_XH = ARM_H + 4   # extra height on tip
    LASER_R = 6.15  # phi 12.3mm for 12mm laser module

    # ── build body ──────────────────────────────────────────────────────────
    hub  = mv(cyl(HUB_R, HUB_H),         0, 0, 0)
    arm  = mv(box(ARM_OX + ARM_L, ARM_W, ARM_H),
                  ARM_OX - ARM_OX, -ARM_W/2, 0)         # shifted so overlap is correct
    # rearrange: arm starts at x = (HUB_R - 4), goes +X
    arm  = Manifold.cube([ARM_OX + ARM_L, ARM_W, ARM_H]
                         ).translate([HUB_R - ARM_OX, -ARM_W/2, 0])
    tip_x = HUB_R + ARM_L - TIP_L + 5
    tip  = Manifold.cube([TIP_L, TIP_W, TIP_XH]
                         ).translate([tip_x, -TIP_W/2, 0])

    body = hub + arm + tip

    # ── holes ───────────────────────────────────────────────────────────────
    # center bore
    body = body - mv(cyl(CLR_R, HUB_H + 2, 64), 0, 0, -1)

    # 4x M2.5 holes at 45/135/225/315 deg
    for deg in [45, 135, 225, 315]:
        rad = math.radians(deg)
        cx, cy = PCD_R * math.cos(rad), PCD_R * math.sin(rad)
        body = body - mv(cyl(HOLE_R, HUB_H + 2, 24), cx, cy, -1)

    # laser hole through tip
    body = body - mv(cyl(LASER_R, TIP_XH + 2, 32), tip_x + TIP_L/2 - 8, 0, -1)

    # lightening holes on arm
    for xc in [HUB_R + 15, HUB_R + 45, HUB_R + 72]:
        body = body - mv(cyl(4.0, ARM_H + 2, 20), xc, 0, -1)

    return body


# ═══════════════════════════════════════════════════════════════════════════
# PART 2: Motor Cradle Base
# Simple half-cylinder pocket for GM3506 body (OD=40mm).
# Motor is held with a zip-tie through two through-slots.
# Base has 4x M4 holes for mounting to a board/tripod.
# ═══════════════════════════════════════════════════════════════════════════
def make_motor_cradle() -> Manifold:
    MOTOR_R = 20.0   # GM3506 OD/2
    GAP     = 0.4    # radial clearance
    POCKET_R = MOTOR_R + GAP
    WALL    = 7.0    # wall thickness around pocket
    OUT_R   = POCKET_R + WALL
    DEPTH   = 26.0   # how deep motor sits in cradle (= motor height)
    BASE_T  = 8.0    # base plate thickness
    BASE_L  = OUT_R * 2 + 10   # base length
    BASE_W  = OUT_R + BASE_T   # base width

    # ── half-cylinder block ─────────────────────────────────────────────────
    # Full cylinder
    outer = cyl(OUT_R, DEPTH, 128)
    # Subtract motor pocket
    outer = outer - mv(cyl(POCKET_R, DEPTH + 2, 128), 0, 0, -1)
    # Cut bottom half away (y < 0 side) to make it a C-cradle
    cut_half = mv(Manifold.cube([OUT_R*2 + 2, OUT_R + 1, DEPTH + 2]),
                  -(OUT_R + 1), -(OUT_R + 1), -1)
    outer = outer - cut_half

    # ── zip-tie through slots (2mm wide, 4mm tall, through wall) ───────────
    for xc in [-(POCKET_R - 2), (POCKET_R - 2)]:
        slot = mv(Manifold.cube([4.0, WALL * 2 + 2, 4.0]),
                  xc - 2, -WALL - 1, (DEPTH/2) - 2)
        outer = outer - slot

    # ── flat base plate ─────────────────────────────────────────────────────
    base = mv(Manifold.cube([BASE_L, BASE_W, BASE_T]),
              -BASE_L/2, -BASE_W, 0)

    # 4x M4 mount holes (through base plate)
    for xi in [-(BASE_L/2 - 10), BASE_L/2 - 10]:
        for zi in [BASE_T/2]:  # just one bolt row centered
            # M4 = 4mm, use 4.5mm clearance hole
            slot_m4 = Manifold.cube([5.0, BASE_W + 2, 5.0]).translate(
                          [xi - 2.5, -BASE_W - 1, BASE_T/2 - 2.5])
            base = base - slot_m4

    # M4 holes (round) in base corners
    for xi in [-(BASE_L/2 - 10), (BASE_L/2 - 10)]:
        for yi in [-(BASE_W - 8)]:
            hole = ry(cyl(2.3, BASE_T + 2, 20), 90).translate([xi, yi, BASE_T/2])
            # Use vertical holes instead
            hole = mv(cyl(2.3, BASE_T + 2, 20), xi, yi, -1)
            base = base - hole

    cradle = outer + base
    return cradle


# ═══════════════════════════════════════════════════════════════════════════
if __name__ == '__main__':
    print('=' * 55)
    print('GM3506 Gimbal Mount Generator v4')
    print('=' * 55)

    ok = True

    print('\n[1/2] Rotor Arm')
    arm = make_rotor_arm()
    ok &= export(arm, 'rotor_arm.stl')

    print('\n[2/2] Motor Cradle')
    cradle = make_motor_cradle()
    ok &= export(cradle, 'motor_shaft_clamp.stl')

    print('\n' + '=' * 55)
    print('RESULT:', 'ALL OK - ready to print!' if ok else 'ISSUES - check above')
    print('=' * 55)
