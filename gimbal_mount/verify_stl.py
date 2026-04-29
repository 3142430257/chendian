"""STL Watertight Checker - ASCII only output"""
import trimesh, os

DIR = r'c:\Users\Administrator\Desktop\chendian\gimbal_mount'

files = {
    'rotor_arm.stl':         {'xr':(80,135), 'yr':(12,30), 'zr':(4,10)},
    'motor_shaft_clamp.stl': {'xr':(50,90),  'yr':(30,80), 'zr':(20,50)},
}

all_ok = True
for fname, spec in files.items():
    p = os.path.join(DIR, fname)
    print(f'\n=== {fname} ===')
    if not os.path.exists(p):
        print('  NOT FOUND'); all_ok = False; continue
    m = trimesh.load(p, force='mesh')
    sz = os.path.getsize(p)/1024
    wt = m.is_watertight
    cn = m.is_winding_consistent
    ext = m.extents.round(1)
    vol = abs(m.volume)
    print(f'  Size       : {sz:.1f} KB')
    print(f'  Faces      : {len(m.faces)}')
    print(f'  Watertight : {"OK" if wt else "FAIL !!! Not printable"}')
    print(f'  Normals    : {"OK" if cn else "WARN inconsistent"}')
    print(f'  Extents XYZ: {ext[0]} x {ext[1]} x {ext[2]} mm')
    xok = spec['xr'][0] <= ext[0] <= spec['xr'][1]
    yok = spec['yr'][0] <= ext[1] <= spec['yr'][1]
    zok = spec['zr'][0] <= ext[2] <= spec['zr'][1]
    print(f'  X range    : {"OK" if xok else f"FAIL expect {spec[chr(120)+"r"]}"}')
    print(f'  Y range    : {"OK" if yok else f"FAIL expect {spec[chr(121)+"r"]}"}')
    print(f'  Z range    : {"OK" if zok else f"FAIL expect {spec[chr(122)+"r"]}"}')
    print(f'  Volume     : {vol:.0f} mm3  (~{vol*1.27/1000:.1f} g PETG)')
    if not (wt and xok and yok and zok):
        all_ok = False

print(f'\nOVERALL: {"PASS - safe to print!" if all_ok else "FAIL - fix needed"}')
