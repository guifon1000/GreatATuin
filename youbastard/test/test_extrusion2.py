from youbastard.io_functions import write_geo
from youbastard.analytic_functions.bezier_functions import chord_function
from youbastard.geo_functions import distance
from youbastard.geometry.Point import Point
from youbastard.geometry.Polyline2D import Polyline2D
from youbastard.geometry.Extrusion import Extrusion
from youbastard.geometry.Spline3D import Spline3D
from youbastard.modelers.profiles.splineProfileMultiParam import Profile
import pygmsh as pg

def lin_fun(value1, value2):
    f = lambda x: value1 + (value2 - value1) * x
    return f

geom = pg.built_in.Geometry()

ndisc = 80
fore_span = 0.5
aft_span = 0.15
fore_max_chord = 0.12
aft_max_chord = 0.05
fore_chordfun = chord_function(0.78)
aft_chordfun = chord_function(0.9)
fore_chordf = lambda s: fore_max_chord * fore_chordfun(s)
aft_chordf = lambda s: aft_max_chord * aft_chordfun(s)
fore_rot_fun = lin_fun(0.01, 0.2)
aft_rot_fun = lin_fun(0.005, 0.12)
fore_profile = Profile(typ = 'fon',par = [0.89  ,0.15 , 0.05 , 0.05 , 0.015],npt = 17) # creation of the 2d profile
aft_profile = Profile(typ = 'fon',par = [0.69  ,0.1 , 0.02 , 0.02 , 0.005],npt = 17) # creation of the 2d profile
fore_pol = fore_profile.polyline(closed = True)
aft_pol = aft_profile.polyline(closed = True)


fore_gen_point = [
        Point([0.,fore_span/float(ndisc),0.]),
        Point([0.0,fore_span/4,0.002]),
        Point([0.,fore_span/2,0.01]),
        Point([-0.02,2.*fore_span/3.,0.015]),
        Point([-0.08,fore_span,-0.02])
            ]

deltaX = -0.6

aft_gen_point = [
        Point([deltaX ,aft_span/float(ndisc),0.]),
        Point([deltaX - 0.005, aft_span/4,-0.002]),
        Point([deltaX ,aft_span/2,-0.005]),
        Point([deltaX - 0.002,2.*aft_span/3.,-0.008]),
        Point([deltaX-0.008,aft_span,0.006])
            ]


fore_generator = Spline3D(fore_gen_point)
fore_wing = Extrusion(fore_pol, fore_generator, scale = fore_chordf, rotate = fore_rot_fun)
fore_wing.pop_to_geom(geom)


if aft_pol.is_closed:
    open_pol = aft_pol[:-1]
else:
    open_pol = aft_pol

npt = len(open_pol)






aft_pol = {
        'extrados': Polyline2D(open_pol[:(npt-1)/2+1]),
        'intrados': Polyline2D(open_pol[(npt-1)/2:]),
        'trailing_edge': Polyline2D([open_pol[-1], open_pol[0]])
        }

aft_generator = Spline3D(aft_gen_point)
aft_wing = Extrusion(aft_pol, aft_generator, scale = aft_chordf, rotate = aft_rot_fun, n = 20)
aft_wing.pop_to_geom(geom)
write_geo('testex',geom)

