from youbastard.geometry.Spline3D import Spline3D
from youbastard.geometry.Point import Point
from youbastard.geometry.Vector import Vector
from youbastard.geometry.Frame import Frame
from youbastard.geometry.Skeleton1D import Skeleton1D
from youbastard.io_functions import write_geo
import pygmsh as pg
from youbastard.modelers.profiles.splineProfileMultiParam import Profile

x = [0., -0.125 , 0.0, .125]
y = [0., 1., 2., 3.]
z = [0., 0.125, 0., -0.125]


pf = Profile(typ = 'fon',par = [0.99,0.1,0.018,0.035,0.001],npt = 11) # creation of the 2d profile
pol = pf.polyline(closed = True)  # TODO : Profile should herit of Polyline_2D

geom = pg.built_in.Geometry()
a_pt = [Point([x[i], y[i], z[i]]) for i in range(len(x))]
from youbastard.geometry.Spline3D import Spline3D
spline = Spline3D(a_pt)

ndisc = 20
fa = spline.frame_array(mode = Vector([0.,0.,1.]), n=ndisc)

for f in fa:
    pp = pol.to_frame(f, scale = 0.2)
    pp.pop_to_geom(geom)


cpol = spline.control_polyline
pol_gen = spline.discretize(ndisc)

sym = spline.symmetrize()

fasym = sym.frame_array(mode = Vector([0.,0.,1.]), n=ndisc)
sym_pol_gen = sym.discretize(ndisc)
for i,f in enumerate(fa):
    bas = f[1]
    new_bas = [[-bas[0][0], -bas[0][1], -bas[0][1]],
               [bas[1][0], bas[1][1], bas[1][1]],
               [bas[2][0], bas[2][1], bas[2][1]]]

    fmirror = Frame([fasym[i][0], new_bas])
    pp = pol.to_frame(fmirror, scale = 0.2)
    pp.pop_to_geom(geom)



cpol.pop_to_geom(geom)
pol_gen.pop_to_geom(geom)
sym.control_polyline.pop_to_geom(geom)
sym_pol_gen.pop_to_geom(geom)



write_geo('toto',geom)


