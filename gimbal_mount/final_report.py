import trimesh
import os

OUT = r'c:\Users\Administrator\Desktop\chendian\gimbal_mount'

files = ['motor_back_mount.stl', 'rotor_arm.stl']

for fname in files:
    p = os.path.join(OUT, fname)
    m = trimesh.load(p, force='mesh')
    
    print("=" * 60)
    print(f"FILE: {fname}")
    print("=" * 60)
    print(f"- Watertight (水密性, 必须为True)      : {m.is_watertight}")
    print(f"- Winding Consistent (法线正常, True)  : {m.is_winding_consistent}")
    print(f"- Empty/Empty space (必须为False)      : {m.is_empty}")
    print(f"- Faces (三角面数量)                   : {len(m.faces)}")
    print(f"- Volume (体积)                        : {abs(m.volume):.1f} mm³")
    
    bbox = m.bounding_box.extents
    print(f"- Bounding Box (长宽高)                : X:{bbox[0]:.1f}mm x Y:{bbox[1]:.1f}mm x Z:{bbox[2]:.1f}mm")
    print(f"- Center of Mass (重心)                : {m.center_mass.round(1)}\n")

