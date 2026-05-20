"""
GM3506 Gimbal Mount — unified generator.
Outputs 2 STLs: bracket.stl + rotor_arm.stl
"""
import sys, subprocess, os, math

for pkg in ['manifold3d', 'trimesh', 'numpy']:
    try: __import__(pkg)
    except ImportError:
        subprocess.check_call([sys.executable, '-m', 'pip', 'install', '--quiet', pkg])

from manifold3d import Manifold
import trimesh
import numpy as np
from trimesh.transformations import rotation_matrix

OUT = os.path.dirname(os.path.abspath(__file__))


def cyl(r, h, seg=64):
    return Manifold.cylinder(h, r, r, seg)


def box(x, y, z):
    return Manifold.cube([x, y, z])


def mv(m, dx=0., dy=0., dz=0.):
    return m.translate([dx, dy, dz])


def m2t(m: Manifold) -> trimesh.Trimesh:
    mesh = m.to_mesh()
    v = np.array(mesh.vert_properties)
    f = np.array(mesh.tri_verts).reshape(-1, 3)
    return trimesh.Trimesh(vertices=v, faces=f)


def rot_x(mesh, deg):
    R = rotation_matrix(math.radians(deg), [1, 0, 0], mesh.centroid)
    mesh.apply_transform(R)


# ---------------------------------------------------------------------------
# PART 1: Motor Back-Mount Bracket
# Bolts to motor BACK face (4x M2.5, PCD 25mm, 45/135/225/315 deg).
# Foot bolts to desk/board with 4x M4 countersunk.
# ---------------------------------------------------------------------------
def make_back_mount():
    PLATE_W = 56.0
    PLATE_H = 56.0
    PLATE_T = 5.0
    FOOT_D = 25.0
    FOOT_T = 5.0
    PCD_R = 12.5
    M25_R = 1.5
    M4_R = 2.3
    FILLET = 4.0

    plate = box(PLATE_W, PLATE_T, PLATE_H - FILLET)
    plate = mv(plate, -PLATE_W / 2, 0, FILLET)
    top = cyl(PLATE_W / 2, PLATE_T, 48)
    top = top.rotate([-90, 0, 0])
    top = mv(top, 0, 0, PLATE_H - FILLET)
    plate = plate + top
    plate = plate.as_original()

    for deg in [45, 135, 225, 315]:
        rad = math.radians(deg)
        cx = PCD_R * math.cos(rad)
        cz = PCD_R * math.sin(rad) + PLATE_H / 2
        hole = cyl(M25_R, PLATE_T + 2, 24)
        hole = hole.rotate([-90, 0, 0])
        plate = plate - mv(hole, cx, -1, cz)

    center = cyl(9.0, PLATE_T + 2, 36)
    center = center.rotate([-90, 0, 0])
    plate = plate - mv(center, 0, -1, PLATE_H / 2)

    plate = plate - mv(box(10, PLATE_T + 2, 12), -5, -1, 0)

    foot = box(PLATE_W, FOOT_D, FOOT_T)
    foot = mv(foot, -PLATE_W / 2, 0, -FOOT_T)

    for x in [-(PLATE_W / 2 - 11), (PLATE_W / 2 - 11)]:
        for y in [8, FOOT_D - 8]:
            th = cyl(M4_R, FOOT_T + 2, 24)
            cb = cyl(4.0, 2.5, 24)
            hole = (th + cb).as_original()
            foot = foot - mv(hole, x, y, -FOOT_T - 1)

    for x in [-(PLATE_W / 2 - 3), (PLATE_W / 2 - 3)]:
        foot = foot + mv(box(8, 15, 12), x - 4, 2, -5)

    return (plate + foot).as_original()


# ---------------------------------------------------------------------------
# PART 2: Rotor Arm
# Bolts to motor FRONT face (4x M2.5, PCD 20mm, 0/90/180/270 deg).
# Payload holes on tip for M3 screws.
# ---------------------------------------------------------------------------
def make_rotor_arm():
    HUB_R = 17.0
    HUB_H = 5.0
    PCD_R = 10.0
    HOLE_R = 1.45  # M2.5 clearance hole, phi 2.9mm
    BORE_R = 3.5   # shaft clearance: shaft=phi5.7, bore=phi7mm, 0.65mm gap
    ARM_REACH = 65.0
    ARM_W = 14.0
    TIP_L = 18.0
    TIP_W = 22.0
    TIP_H = HUB_H + 3.0

    hub = cyl(HUB_R, HUB_H, 96)
    arm_x0 = HUB_R - 4
    arm = mv(box(ARM_REACH - HUB_R + 4, ARM_W, HUB_H), arm_x0, -ARM_W / 2, 0)
    tip_x = ARM_REACH - TIP_L
    tip = mv(box(TIP_L, TIP_W, TIP_H), tip_x, -TIP_W / 2, 0)

    body = hub + arm + tip
    body = body - mv(cyl(BORE_R, HUB_H + 2, 48), 0, 0, -1)

    for deg in [0, 90, 180, 270]:
        rad = math.radians(deg)
        body = body - mv(cyl(HOLE_R, HUB_H + 2, 24),
                         PCD_R * math.cos(rad), PCD_R * math.sin(rad), -1)

    for x in [ARM_REACH - 14, ARM_REACH - 7]:
        body = body - mv(cyl(1.75, TIP_H + 2, 20), x, 0, -1)

    for xc in [HUB_R + 10, HUB_R + 25, HUB_R + 40]:
        body = body - mv(cyl(3.5, HUB_H + 2, 20), xc, 0, -1)

    return body.as_original()


# ---------------------------------------------------------------------------
# Generate + Render
# ---------------------------------------------------------------------------
def generate_all():
    bracket_stl = m2t(make_back_mount())
    arm_stl = m2t(make_rotor_arm())

    for name, obj in [('bracket.stl', bracket_stl), ('rotor_arm.stl', arm_stl)]:
        p = os.path.join(OUT, name)
        obj.export(p)
        ext = obj.extents
        vol = abs(obj.volume) * 1.27 / 1000
        print(f"  {name}: {ext[0]:.0f} x {ext[1]:.0f} x {ext[2]:.0f} mm  ~{vol:.1f}g PETG")

    print("\n  bracket.stl  -> motor BACK face  (4x M2.5, PCD 25mm)")
    print("  rotor_arm.stl -> motor FRONT face (4x M2.5, PCD 20mm)")

    # --- render copies (transformed for assembly view) ---
    bracket = bracket_stl.copy()
    arm = arm_stl.copy()
    motor = trimesh.creation.cylinder(radius=20.0, height=25.8, sections=64)

    for m, c in [(bracket, [80, 90, 100, 255]),
                  (arm, [70, 80, 90, 255]),
                  (motor, [160, 160, 165, 255])]:
        m.visual.face_colors = c

    PLATE_CENTER_Z = 28.0
    PLATE_THICK_Y = 5.0
    MOTOR_LEN = 25.8
    HUB_HALF = 2.5

    # Bracket: standing upright (natural orientation), no rotation
    # plate is vertical XZ at y=0..5, foot extends +Y at bottom z=-5..0

    # Motor: rotate 90° around X so axis runs along Y (horizontal)
    rot_x(motor, 90)
    motor_y_center = PLATE_THICK_Y + MOTOR_LEN / 2
    motor.apply_translation([0, motor_y_center, PLATE_CENTER_Z])

    # Rotor arm: rotate 90° around X so flat hub face is vertical XZ
    rot_x(arm, 90)
    arm_y = PLATE_THICK_Y + MOTOR_LEN + HUB_HALF
    arm.apply_translation([0, arm_y, PLATE_CENTER_Z])

    _render(bracket, arm, motor,
            dim_arm=arm_stl, dim_bracket=bracket_stl)


def _render(bracket, arm, motor, dim_arm=None, dim_bracket=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    from matplotlib.patches import Patch

    parts = [
        (bracket, '#5a657a', 'Bracket (back mount)'),
        (arm, '#3d4555', 'Rotor Arm'),
        (motor, '#b0b5c0', 'Motor (ref)'),
    ]

    # Figure 1: 4-view exploded
    fig = plt.figure(figsize=(20, 15))
    views = [
        (221, "Isometric view", (28, -40)),
        (222, "Front view", (0, 90)),
        (223, "Side view (motor axis)", (0, 0)),
        (224, "Top view", (90, 0)),
    ]

    for pos, title, (elev, azim) in views:
        ax = fig.add_subplot(pos, projection='3d')
        for mesh, color, _ in parts:
            verts = mesh.vertices
            faces = mesh.faces
            tri_verts = verts[faces]
            pc = Poly3DCollection(tri_verts, alpha=0.75, facecolor=color,
                                   edgecolor='#222222', linewidth=0.06)
            ax.add_collection3d(pc)

        all_v = np.vstack([m.vertices for m, _, _ in parts])
        mid = all_v.mean(axis=0)
        r = (all_v.max(axis=0) - all_v.min(axis=0)).max() * 0.7

        ax.set_xlim(mid[0] - r, mid[0] + r)
        ax.set_ylim(mid[1] - r, mid[1] + r)
        ax.set_zlim(mid[2] - r, mid[2] + r)
        ax.set_title(title, fontsize=13, fontweight='bold')
        ax.view_init(elev=elev, azim=azim)
        ax.set_axis_off()

    legend_elements = [Patch(facecolor=c, edgecolor='#222', label=l)
                       for _, c, l in parts]
    fig.legend(handles=legend_elements, loc='lower center', ncol=3,
               fontsize=11, frameon=False)

    plt.suptitle('GM3506 Gimbal Mount', fontsize=17, fontweight='bold', y=0.97)
    plt.tight_layout(rect=[0, 0.05, 1, 0.94])
    png1 = os.path.join(OUT, 'assembly_view.png')
    plt.savefig(png1, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close()
    print(f"  Render: {png1}")

    # Figure 2: dimensioned top views (use unrotated meshes)
    dim_arm = dim_arm if dim_arm is not None else arm
    dim_bracket = dim_bracket if dim_bracket is not None else bracket
    fig2, axes = plt.subplots(1, 2, figsize=(12, 6))
    for ax, mesh, name, color in zip(
        axes,
        [dim_arm, dim_bracket],
        ['Rotor Arm', 'Back-Mount Bracket'],
        ['#3d4555', '#5a657a']
    ):
        verts = mesh.vertices
        faces = mesh.faces
        tri_verts = verts[faces]
        ax.set_aspect('equal')
        for tri in tri_verts:
            poly = plt.Polygon(tri[:, :2], closed=True, facecolor=color,
                               edgecolor='#222', linewidth=0.3, alpha=0.85)
            ax.add_patch(poly)

        ext = mesh.extents
        cx, cy = mesh.centroid[:2]
        w, h = ext[0], ext[1]
        margin = max(w, h) * 0.18

        ax.set_xlim(cx - w / 2 - margin, cx + w / 2 + margin)
        ax.set_ylim(cy - h / 2 - margin, cy + h / 2 + margin)

        y_dim = cy - h / 2 - margin * 0.55
        ax.annotate('', xy=(cx - w / 2, y_dim), xytext=(cx + w / 2, y_dim),
                     arrowprops=dict(arrowstyle='<->', color='#e53e3e', lw=1.5))
        ax.text(cx, y_dim - margin * 0.15, f'{w:.0f} mm',
                ha='center', va='top', fontsize=10, color='#e53e3e', fontweight='bold')

        x_dim = cx + w / 2 + margin * 0.55
        ax.annotate('', xy=(x_dim, cy - h / 2), xytext=(x_dim, cy + h / 2),
                     arrowprops=dict(arrowstyle='<->', color='#3182ce', lw=1.5))
        ax.text(x_dim + margin * 0.15, cy, f'{h:.0f} mm',
                ha='left', va='center', fontsize=10, color='#3182ce', fontweight='bold')

        ax.set_title(f'{name}\n{w:.0f} x {h:.0f} x {ext[2]:.0f} mm',
                     fontsize=12, fontweight='bold')
        ax.set_xticks([])
        ax.set_yticks([])

    plt.suptitle('Part Dimensions (Top View)', fontsize=16, fontweight='bold')
    plt.tight_layout(rect=[0, 0, 1, 0.93])
    png2 = os.path.join(OUT, 'dimensions.png')
    plt.savefig(png2, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close()
    print(f"  Render: {png2}")


if __name__ == '__main__':
    generate_all()
