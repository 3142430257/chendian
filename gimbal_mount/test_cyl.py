from manifold3d import Manifold
import trimesh
import numpy as np

def m_to_trimesh(m: Manifold) -> trimesh.Trimesh:
    mesh = m.to_mesh()
    v = np.array(mesh.vert_properties)
    f = np.array(mesh.tri_verts).reshape(-1, 3)
    return trimesh.Trimesh(vertices=v, faces=f)

m = Manifold.cylinder(10.0, 5.0, 5.0, 36)
t = m_to_trimesh(m)
print("cylinder bounds (Z):", np.min(t.vertices[:, 2]), "to", np.max(t.vertices[:, 2]))
