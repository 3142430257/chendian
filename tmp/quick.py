import json, sys
fs = [json.loads(l) for l in open(sys.argv[1])]
print(f"frames: {len(fs)}")
print(f"start POS: {fs[0]['pos_act']:+.2f}")
print(f"end POS: {fs[-1]['pos_act']:+.2f}")
print(f"delta POS: {fs[-1]['pos_act'] - fs[0]['pos_act']:+.2f}")
# 看 ALIGN 期间 POS-vs-time
align_fs = [f for f in fs if f.get('tag') == 'a']
print(f"ALIGN frames: {len(align_fs)}")
if align_fs:
    print(f"  ALIGN start POS: {align_fs[0]['pos_act']:+.2f}, end: {align_fs[-1]['pos_act']:+.2f}")
