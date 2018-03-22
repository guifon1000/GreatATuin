
from youbastard.io_functions import pretty_print, write_polylines_sets
from youbastard.geo_functions import parameter_frame
from youbastard.geometry.Triangulation import Triangulation, read_msh_file, merge, read_json_file
from youbastard.geometry.Sphere import Sphere
from youbastard.geometry.Extrusion import Extrusion
from youbastard.geometry.Box import Box
from youbastard.geometry.Point import Point
from youbastard.modelers.planet.Planet import Planet
from youbastard.modelers.profiles.splineProfileMultiParam import Profile
from scipy import interpolate
import numpy as np
idtest = 1
pretty_print('TEST n.' + str(idtest) + ': IMPORT JSON TRIANGULATION')


d = read_json_file('../samples/icosahedron.json')
print d['faces']

idtest += 1
pretty_print('TEST n.' + str(idtest) + ': SPHERES')

d.reorient_convex()
d.translate((1.,0.,-2.))
d = d.refine_2()
d.write_obj_file('refined_icosahedron.obj')
s1 = Sphere(refin = 0, center = (1.,1.,1.), radius = 0.3)
s2 = Sphere(refin = 1, center = (0.,-4.,2.), radius = 0.1)
s3 = Sphere(refin = 2, center = (0.,0.,0.), radius = 0.7)
p1 = Planet(refin = 1, center = (-3.,-5.,-3.), radius = 0.8)
s1.write_obj_file('sphere1.obj')
s1.write_fms_file('sphere')
s2.write_obj_file('sphere2.obj')
s3.write_obj_file('sphere3.obj')
p1.write_obj_file('planet1.obj')

idtest += 1
pretty_print('TEST n.' + str(idtest) + ': EXTRUSION ALONG 3D CURVE')

vtk_elements = []

pf = Profile(typ = 'fon',par = [0.99,0.1,0.018,0.035,0.001],npt = 11) # creation of the 2d profile
pol = pf.polyline(closed = True)  # TODO : Profile should herit of Polyline_2D

print type(pf)
# control points of the generatrix
global_scale = 0.1


write_polylines_sets('profile', profile = pol)


x = [-6., -5., 0., 5., 6.]
y = [0., -0.5, -1.5, -0.5, 0.]
z = [0.0, 0.,0., 0., -0.00]


import pygmsh as pg
geom = pg.built_in.Geometry()
a_pt = [Point([x[i], y[i], z[i]]) for i in range(len(x))]
from youbastard.geometry.Spline3D import Spline3D
from youbastard.io_functions import write_geo
spline = Spline3D(a_pt)


def extrude_along_spline(polyline2d, spline, scale = lambda x: 1., rotation = lambda x: 0., resol = 20):
    print 'coucou'
    a_pol3d = None
    return a_pol3d




x = [v * global_scale for v in x]
y = [v * global_scale for v in y]
z = [v * global_scale for v in z]


# tck, u represent the parametric 3d curve
tck, u = interpolate.splprep([x,y,z], s=3)

frames = []
scales = []

for s in np.linspace(0., 1., num = 50):
    frame = parameter_frame(tck, s, mode = 'Znat')
    scale =  global_scale * (0.5 + 5.* s * (1. - s))
    scales.append(scale)
    vtk_elements.append(frame)
    frames.append(frame)

extrusion = Extrusion(pol, frames, scales = scales, close_caps = True, mode = 'profile')

revex = extrusion.reverse()
extrusion.write_fms_file('extrusion')
box = Box(Point((0.,-1.,0.)), 7.,7.,4.)
extrusion.write_fms_file('box')
mer = merge( box, revex)
mer.write_obj_file('merge.obj')
print 'writing the obj file for extrusion'
extrusion.write_obj_file('extrusion.obj')
mer.write_fms_file('merge')
#vtk_visu(vtk_elements)
#write_geo('test_'+str(idtest), geom)
