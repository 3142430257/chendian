from manifold3d import Manifold
m = Manifold.cube([10, 10, 10])
print('box:', m.bounds())
c = Manifold.cylinder(10, 5, 5)
print('cyl:', c.bounds())
