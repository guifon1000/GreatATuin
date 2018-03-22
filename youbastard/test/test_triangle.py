import numpy as np
from youbastard.geo_functions import (is_on_plane, intersect_2_segments, 
        matrix_to_quaternion, intersect_2_lines, distance)
from youbastard.io_functions import pretty_print
from youbastard.geometry.Point import Point
from youbastard.geometry.Triangle import Triangle
from youbastard.geometry.Line import Line
from youbastard.geometry.Vector import Vector
from youbastard.geometry.Plane import Plane
from youbastard.geometry.Segment import Segment
from youbastard.geometry.TreeNode import TreeNode
from youbastard.geometry.Quaternion import Quaternion
idtest = 1
pretty_print('TEST n.'+str(idtest) + ': TRIANGLE')


# create three points
p1 = Point((0.01,0.0,0.2))
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
print 'normal = '+str(t.normal)

print is_on_plane(t.circumcenter, pl)
print is_on_plane(t.cg, pl)
print is_on_plane(Point((0.,0.,0.)), pl)



idtest += 1
pretty_print('TEST n.' + str(idtest) + ': LINES')


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
pretty_print('TEST n.' + str(idtest) + ': POLAR PARTITION OFFSET')

data = {'points': [Point([0., 0., 0.]), 
                    Point([10., 10., 0.]), 
                    Point([-14., -6., 0.]),
                    Point([-12., 6.8, 0.]),
                    Point([11.2, -2.8, 0.]),
                    Point([-4., 6., 0.])],
        'walls': [[0,1,0.1], [0,2,0.25], [0,3,0.08], [0,4,0.06], [0,5,0.2]]}


test_sec = None
all_tree_nodes = []
all_thicknesses = []

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
