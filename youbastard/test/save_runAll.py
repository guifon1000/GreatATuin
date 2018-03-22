import sys
sys.path.append('../')
from classes.Point import Point
from classes.Line import Line
from classes.Triangle import Triangle
from classes.Triangulation import Triangulation, read_msh_file, merge, read_json_file
from classes.Vector import Vector
from classes.Sphere import Sphere
from classes.Frame import Frame
from classes.Quaternion import Quaternion
from classes.Extrusion import Extrusion
from classes.Box import Box
from classes.Plane import Plane
import pygmsh as pg
from modelers.profiles.splineProfileMultiParam import  Profile
from modelers.planet.Planet import  Planet
from scipy import interpolate
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
from functions import parameter_frame, matrix_to_quaternion, vtk_visu, write_geo, write_fms_file, angle, cross, is_on_line, is_on_plane, intersect_2_lines, intersect_2_segments, get_parameter, distance
from io_functions import write_polylines_sets 

def pretty_print(string, symbol = '-'):
    s = ''
    for i in range(79):
        s += symbol
    s += '\n'
    lstr = len(string) + 2 
    s2 = ''
    if lstr >= 79:
        s2 = string + '\n'
    else:
        si = ' ' + string + ' '
        n1 = int((79 - lstr )/2)
        n2 = 79 - len(si) - n1
        s2 = ''
        for i in range(n1): s2 += symbol
        s2 += si
        for i in range(n2): s2 += symbol 
        s2 += '\n'
    print( s + s2 + s)







idtest = 1
pretty_print('TEST n.'+str(idtest) + ': TRIANGLE')


# create three points
p1 = Point((0.001,0.0,0.))
p2 = Point((0.23,0.5,0.))
p3 = Point((1.,0.0,0.))

points = [p1, p2, p3]

geo_points = []
# create a triangle
t = Triangle(points)

pl = Plane(points)





print ' ----- A TRIANGLE ------ '
print t
print 'attributes'
print 'cog = '+str(t.cg)
print 'normal = '+str(t.normal)

print ' REVERSE ---------> '
t = -t
print t
print 'attributes'
print 'cog = '+str(t.cg)
print 'circumcenter = '+str(t.circumcenter)
#t.cg.pop_to_geom(geom)
print 'normal = '+str(t.normal)

print is_on_plane(t.circumcenter, pl)
print is_on_plane(t.cg, pl)
print is_on_plane(Point((0.,0.,0.)), pl)



geom = pg.built_in.Geometry()

gp0 = points[0].pop_to_geom(geom)
gp1 = points[1].pop_to_geom(geom)
gp2 = points[2].pop_to_geom(geom)

cc = t.circumcenter
gp3 = cc.pop_to_geom(geom)
arc1 = geom.add_circle_arc(gp0, gp3, gp1)
arc1 = geom.add_circle_arc(gp1, gp3, gp2)
arc1 = geom.add_circle_arc(gp2, gp3, gp0)

v = t.normal
print "raw vector :"+str(v)
print "norm 2 : "+str(v.norm2)
print "norm 1 : "+str(v.norm)
print "unit vector : "+str(v.unit())

point_line = points[0]
vector_line = Vector([points[1][i] - points[0][i] for i in range(3)])
line = Line([point_line, vector_line])
mid = Point([0.5 * (points[0][i] + points[1][i]) for i in range(3)])

print is_on_line(mid, line)
for i in range(100):
    s = -50. + float(i)
    pt = line.parameter_point(s)
    pt.pop_to_geom(geom)



write_geo('test_'+str(idtest), geom)


idtest += 1
pretty_print('TEST n.' + str(idtest) + ': LINES')


geom = pg.built_in.Geometry()

p1 = points[0]
v1 = Vector([points[1][i] - points[0][i] for i in range(3)])
l1 = Line((p1,v1))
p2 = points[2]
v2 = Vector([points[1][i] - points[2][i] for i in range(3)])
l2 = Line((p2,v2))
print '======='
print intersect_2_lines(l1, l2)
print '--------------'
print points[1]
print '======='
write_geo('test_'+str(idtest), geom)


from classes.Segment import Segment, OffsetSegment
idtest += 1
pretty_print('TEST n.' + str(idtest) + ': INTERSECT 2 SEGMENTS')
p1 = Point([0.,0.,0.001])
p2 = Point([1.,1.,0.])
p3 = Point([0.,1.,0.])
p4 = Point([1.,0.,0.])
seg1 = Segment([p2, p1])
seg2 = Segment([p3, p4])
print intersect_2_segments(seg1, seg2)




idtest += 1
pretty_print('TEST n.' + str(idtest) + ': 2 SEGMENTS POLYLINE OFFSET')
geom = pg.built_in.Geometry()
data = {'points': [Point([0., 0., 0.]), 
                   Point([3., 0., 0.]), Point([2., 1.25, 0.]), Point([1.85, 2.3,0.])],
        'walls': [[0,2], [2,1], [3,2]],
        'thickness': [0.15, 0.185, 0.11]}
data1 = {'points': [Point([0., 0., 0.]), Point([3., 0., 0.]), Point([2., 1.25, 0.])],
        'walls': [[0,2], [2,1]],
        'thickness': [0.15, 0.185]}

print data
epsilon = 0.0001



offsets = []


for iwall, wall in enumerate(data['walls']):
    # the directing vector
    p1 = data['points'][wall[0]]
    p2 = data['points'][wall[1]]
    seg = Segment([p1, p2])
    thickness = data['thickness'][iwall]
    offsets.append(OffsetSegment(seg, thickness))


linked_walls = []

for i,walli in enumerate(data['walls']):
    #linked_points = walli
    neig = []
    for j,wallj in enumerate(data['walls']):
        if i!=j:
            if (walli[0] in wallj) or (walli[1] in wallj):
                neig.append(j)
    linked_walls.append(neig)

print '-----'
print linked_walls





for iwall,connected_walls in enumerate(linked_walls):
    offsets[iwall].update_candidates([offsets[i] for i in connected_walls])



for w in offsets:
    #w.pop_to_geom(geom)
    for p in w.candidates_1:
        pcorner = p.pop_to_geom(geom)
        pmid = w.l1[0].pop_to_geom(geom)
        geom.add_line(pmid, pcorner)
    for p in w.candidates_2:
        pcorner = p.pop_to_geom(geom)
        pmid = w.l2[0].pop_to_geom(geom)
        geom.add_line(pmid, pcorner)
write_geo('test_'+str(idtest), geom)

idtest += 1
pretty_print('TEST n.' + str(idtest) + ': POLAR PARTITION OFFSET')

data = {'points': [Point([0., 0., 0.]), 
                    Point([10., 10., 0.]), 
                    Point([-14., -6., 0.]),
                    Point([-12., 6.8, 0.]),
                    Point([11.2, -2.8, 0.]),
                    Point([-4., 6., 0.])],
        'walls': [[0,1,0.1], [0,2,0.25], [0,3,0.08], [0,4,0.06], [0,5,0.2]]}
geom = pg.built_in.Geometry()


test_sec = None
all_tree_nodes = []
all_thicknesses = []

from classes.TreeNode import TreeNode
for ipoint,point in enumerate(data['points']):
    sector = [point]
    for iwall,wall in enumerate(data['walls']):
        if ipoint in wall[:2]:
            nex=None
            if wall[0]==ipoint:
                nex = data['points'][wall[1]]
            else:
                nex = data['points'][wall[0]]
            length = distance(point, nex)
            vec = Vector([nex[i]-point[i] for i in range(3)]).unit()
            half = Point([point[i] + 0.5 * length * vec[i] for i in range(3)])
            thick = float(wall[2])
            sector.append([half, thick])
    all_tree_nodes.append(TreeNode([sector[0]] + [ secp[0] for secp in sector[1:]]))
    all_thicknesses.append([ secp[1] for secp in sector[1:]])


polylines = []


for itreenode, treenode in enumerate(all_tree_nodes): 
    thck = all_thicknesses[itreenode]
    treenode.set_thicknesses(thck)
    pl = treenode.offset()
    polylines.append(pl)
    #pl.pop_to_geom(geom)
write_geo('TreeNode', geom)

idtest += 1
pretty_print('TEST n.' + str(idtest) + ': SIMPLIFIED HOUSE 2D')

data = {'points': [Point([-5., 2.5, 0.]), 
                    Point([5., 2.5, 0.]), 
                    Point([5., -2.5, 0.]),
                    Point([-5., -2.5, 0.])],
        'walls': [[0,1], [1,2], [2,3], [3,0]]}
polylines = []
all_tree_nodes = []
for ipoint,point in enumerate(data['points']):
    sector = [point]
    for iwall,wall in enumerate(data['walls']):
        if ipoint in wall[:2]:
            nex=None
            if wall[0]==ipoint:
                nex = data['points'][wall[1]]
            else:
                nex = data['points'][wall[0]]
            length = distance(point, nex)
            vec = Vector([nex[i]-point[i] for i in range(3)]).unit()
            half = Point([point[i] + 0.5 * length * vec[i] for i in range(3)])
            sector.append([half, thick])
    all_tree_nodes.append(TreeNode([sector[0]] + [ secp[0] for secp in sector[1:]]))





for itreenode, treenode in enumerate(all_tree_nodes): 
    #thck = all_thicknesses[itreenode]
    #treenode.set_thicknesses(thck)
    pl = treenode.offset(default_thickness = 0.2)
    polylines.append(pl)
    pl.pop_to_geom(geom)



write_geo('SimpleHouse', geom)

write_polylines_sets('zob', walls=polylines)
idtest += 1
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
write_fms_file('sphere', s1)
s2.write_obj_file('sphere2.obj')
s3.write_obj_file('sphere3.obj')
p1.write_obj_file('planet1.obj')

idtest += 1
pretty_print('TEST n.' + str(idtest) + ': EXTRUSION ALONG 3D CURVE')

geom = pg.built_in.Geometry()
vtk_elements = []

pf = Profile(typ = 'fon',par = [0.99,0.1,0.018,0.035,0.001],npt = 101) # creation of the 2d profile
pol = pf.polyline(closed = True)  # TODO : Profile should herit of Polyline_2D

print type(pf)
# control points of the generatrix
global_scale = 0.1


write_polylines_sets('profile', profile = pol)


x = [-6., -5., 0., 5., 6.]
y = [0., -0.5, -1.5, -0.5, 0.]
z = [0.0, 0.,0., 0., -0.00]


x = [v * global_scale for v in x]
y = [v * global_scale for v in y]
z = [v * global_scale for v in z]


# tck, u represent the parametric 3d curve
tck, u = interpolate.splprep([x,y,z], s=3)

frames = []
scales = []

for s in np.linspace(0., 1., num = 5):
    frame = parameter_frame(tck, s, mode = 'Znat')
    scale =  global_scale * (0.5 + 5.* s * (1. - s))
    scales.append(scale)
    vtk_elements.append(frame)
    frames.append(frame)

extrusion = Extrusion(pol, frames, scales = scales, close_caps = True, mode = 'profile')

revex = extrusion.reverse()
write_fms_file('extrusion', extrusion)
box = Box(Point((0.,-1.,0.)), 7.,7.,4.)
write_fms_file('box', box)
mer = merge( box, revex)
mer.write_obj_file('merge.obj')
print 'writing the obj file for extrusion'
extrusion.write_obj_file('extrusion.obj')
write_fms_file('merge', mer)
vtk_visu(vtk_elements)
write_geo('test_'+str(idtest), geom)

idtest += 1
pretty_print('TEST n.' + str(idtest) + ': QUATERNION')

q1 = Quaternion((1., 0.5, 0.6, 0.9))
print 'Q1 : '
print q1
print 'Q1 CONJUGATE : '
print q1.conjugate
print 'Q1 NORM : '
print abs(q1)
print 'Q1 INVERSE : '
print q1.inverse()
print 'Q1 * Q1^(-1)'
print q1 * q1.inverse()
print 'Q1 -> matrix'
print q1.to_matrix()

mat = np.zeros((3,3))
mat = np.array([ [0., 1., 0. ] , [-1, 0., 0.] , [0., 0., 1.] ])

print '=========================='
print 'MATRIX :'
print mat
print '=========================='
print 'MATRIX -> QUATERNION :'
q =  matrix_to_quaternion(mat)
print q 
print '=========================='
print 'QUATERNION -> MATRIX :'
print q.to_matrix()

q2 = Quaternion((4., 0.25, 0.26, 0.49))
print q1 * q2


idtest += 1
pretty_print('TEST n.' + str(idtest) + ': ARCS WITH BULGE')
a = 1.
points = [
         Point((0,0,0)),
         Point((a/2, a*np.sqrt(3.)/2, 0.)),
         Point((a/2, a*np.sqrt(3.)/4, 0.)) #,
         #Point((a, 0., 0))
         ]

segments = [(0,1), (1,2), (2,0), (0,3), (3,1), (3,2) ]
segments = [(0,1), (1,2)]

class Segment(dict):
    def __init__(self, start_point, end_point, bulge = 0.):
        self['start_point'] = start_point
        self['end_point'] = end_point
        self['bulge'] = point
    #def pop_to_geom(self, geom):
        
            
            
#class PolylineCurve(list):
        

def center_arc(pt1, pt2, bulge):
    if bulge > 0.:
        inc_angle = 4. * np.arctan(bulge)
    elif bulge < 0.:
        inc_angle = -4. * np.arctan(bulge)
    chord = Vector([pt2[i] - pt1[i] for i in range(3)])
    mid = Point([0.5 * (pt1[i] + pt2[i]) for i in range(3) ])
    vec = (chord.norm * 0.5 * bulge * cross(chord, Vector((0., 0., 1.))).unit())
    summit = Point([mid[i] + vec[i] for i in range(3) ])
    radius = chord.norm / (2. * np.sin(inc_angle/2.))
    vec = radius * Vector([mid[i] - summit[i] for i in range(3)]).unit()
    center = Point([summit[i] + vec[i] for i in range(3) ])
    return center



geom = pg.built_in.Geometry()
center1 = center_arc(points[0], points[1], 0.1)
center2 = center_arc(points[0], points[1], -0.4)
lcar = 0.1
p0 = geom.add_point(points[0], lcar)
p1 = geom.add_point(points[1], lcar)
c1 = geom.add_point(center1, lcar)
c2 = geom.add_point(center2, lcar)
arc1 = geom.add_circle_arc(p0, c1, p1)
arc2 = geom.add_circle_arc(p0, c2, p1)

loop = geom.add_line_loop([arc1, -arc2])
sf = geom.add_plane_surface(loop)
#geom.add_circle_arc(p0, p1, p2)

write_geo('test_'+str(idtest), geom)



idtest += 1
pretty_print('TEST n.' + str(idtest) + ': PLANAR EXTRUSION')
thickness = 0.1
geom = pg.built_in.Geometry()
po1 = Point((0., 0., 0.))
po2 = Point((1., 1., 0.))
po3 = Point((2., 1., 0.))

vec = Vector([po2[i] - po1[i] for i in range(3)]).unit()
trans = cross([0.,0.,1.], vec).unit()

start_1 = Point([po1[i] + 0.5 * thickness * trans[i] for i in range(3) ])
start_2 = Point([po1[i] - 0.5 * thickness * trans[i] for i in range(3) ])

p1 = po1.pop_to_geom(geom)
p2 = po2.pop_to_geom(geom)
s1 = start_1.pop_to_geom(geom)
s2 = start_2.pop_to_geom(geom)
lwall = geom.add_line(p1,p2)
lthick = geom.add_line(s1,s2)
wall = Vector([po2[i] - po1[i] for i in range(3)])
geom.extrude(lthick, translation_axis=wall, rotation_axis=None, point_on_axis=po1, angle=0.)
write_geo('test_'+str(idtest), geom)

p1 = Plane((0.,0.,0.,0.))
p2 = Plane((po1, po2, po3))
