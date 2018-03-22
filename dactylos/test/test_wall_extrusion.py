import json
from dactylos.cad_functions import extrude_polyline2d
from dactylos.cad_functions import boolean_union
from youbastard.geometry.Point import Point
from youbastard.geometry.Vector import Vector
from youbastard.geometry.Frame import Frame
from youbastard.geometry.Plane import Plane
from youbastard.geometry.Polyline2D import Polyline2D

d = json.load(open('simple_house.json','r'))

plane = Plane([Point([0., 0., 0.]), Vector([0., 0., 1.])])

frame = Frame([[0., 0., 0.],[1., 0., 0.],[0., 1., 0.], [0., 0., 1.]])




shapes = []



for pol in d['walls']:
    pol0 = Polyline2D([[p[0],p[1]] for p in pol])
    shape = extrude_polyline2d(pol0, frame, 10.)
    shapes.append(shape)


union = boolean_union(shapes)


