import trimesh, os
OUT = r'c:\Users\Administrator\Desktop\chendian\gimbal_mount'
results = []
for fname in ['rotor_arm.stl', 'motor_shaft_clamp.stl']:
    p = os.path.join(OUT, fname)
    if not os.path.exists(p):
        results.append(f'{fname}: NOT FOUND')
        continue
    m = trimesh.load(p, force='mesh')
    ext = m.extents.round(1).tolist()
    vol = abs(m.volume)
    lines = [
        f'=== {fname} ===',
        f'  Watertight: {m.is_watertight}',
        f'  Normals   : {m.is_winding_consistent}',
        f'  Extents   : {ext[0]} x {ext[1]} x {ext[2]} mm',
        f'  Volume    : {vol:.0f} mm3  ({vol*1.27/1000:.1f} g PETG)',
        f'  Faces     : {len(m.faces)}',
    ]
    results.extend(lines)

report = os.path.join(OUT, 'stl_report.txt')
with open(report, 'w', encoding='ascii', errors='replace') as f:
    f.write('\n'.join(results) + '\n')
print('Report written to', report)
