import trimesh
import os

OUT = r'c:\Users\Administrator\Desktop\chendian\gimbal_mount'

for fname in ['motor_back_mount.stl', 'rotor_arm.stl']:
    p = os.path.join(OUT, fname)
    m = trimesh.load(p, force='mesh')
    vol = m.volume
    # If volume is negative, all normals point inward -> flip
    if vol < 0:
        m.invert()
    # Also fix any remaining inconsistent normals
    trimesh.repair.fix_normals(m, multibody=False)
    # Export
    m.export(p)
    print(fname, '| vol:', round(abs(m.volume), 1),
          '| wt:', m.is_watertight,
          '| consistent:', m.is_winding_consistent)
