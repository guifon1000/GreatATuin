from dactylos.modelers.house import Wall, Opening, rectangle, BoxOnWall
from dactylos.cad_functions import (show_shapes, extrude_polyline2d, boolean_union, 
                                    general_fuse_algorithm, general_split_algorithm,
                                    face_polyline, big_box, shape_array)
from youbastard.geometry.Vector import Vector
from youbastard.geometry.Point import Point
from youbastard.geometry.Frame import Frame
from youbastard.geometry.TreeNode import TreeNode
from youbastard.geometry.Polyline2D import Polyline2D
from youbastard.geo_functions import distance
import json

data = {'points': [Point([-5.,  2.5, 0.]),
                   Point([ 5.,  2.5, 0.]), 
                   Point([ 5., -2.5, 0.]), 
                   Point([-5., -2.5, 0.])], 
        'walls': [[0,1], [1,2], [2,3], [3,0]]}
data = {}
thick = 0.15

romarine = json.load(open('romarine_cad.json','r'))
print romarine
data['points'] = []
for p in romarine['points']:
    data['points'].append(Point(p))
data['walls'] = romarine['walls']


total_height = 3.
    
floor_frame = Frame([[0.,0.,0.], [[0, 0., 1.], [1., 0., 0.], [0., 1., 0.]]])
ceiling_frame = Frame([[0.,0., total_height], [[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]]])


horiz = Polyline2D([[-30., -30.],
                   [30., -30.],
                   [30., 30.],
                   [-30., 30.]], closed = True)


floor = extrude_polyline2d(horiz, floor_frame, -0.5)
ceiling = extrude_polyline2d(horiz, ceiling_frame, 0.1)


polylines = []
all_tree_nodes = []
thick = 0.15
for ipoint,point in enumerate(data['points']):
    sector = [point]
    for iwall,wall in enumerate(data['walls']):
        if ipoint in wall:
            nex=None
            if wall[0]==ipoint:
                nex = data['points'][wall[1]]
            elif wall[1]==ipoint:
                nex = data['points'][wall[0]]
            length = distance(point, nex)
            vec = Vector([nex[i]-point[i] for i in range(3)]).unit()
            half = Point([point[i] + 0.5 * length * vec[i] for i in range(3)])
            sector.append([half, thick])
    all_tree_nodes.append(TreeNode([sector[0]] + [ secp[0] for secp in sector[1:]]))

for itreenode, treenode in enumerate(all_tree_nodes): 
    #thck = all_thicknesses[itreenode]
    #treenode.set_thicknesses()
    pl = treenode.offset(default_thickness = thick)
    polylines.append(pl)


shapes = []

frame = Frame([[0.,0.,0.],[[0.,0.,1.],[1.,0.,0.],[0.,1.,0.]]])



for pol in polylines:
    pol0 = Polyline2D([[p[0],p[1]] for p in pol])
    shape = extrude_polyline2d(pol0, frame, total_height)
    shapes.append(shape)

#union = boolean_union(shapes)

import matplotlib.pyplot as plt

for i,p in enumerate(data['points']):
    plt.scatter(p[0],p[1], s = 2, c='k')
    plt.text(p[0],p[1]+0.25,str(i))


for w in data['walls']:
    plt.plot([data['points'][w[i]][0] for i in range(2)],
            [data['points'][w[i]][1] for i in range(2)],'r')


for pol in polylines:
    plt.plot([p[0] for p in pol], [p[1] for p in pol], 'k')
plt.axis('equal')
plt.show()


pts = data['points']
walls = data['walls']

height = 3.
vertical = Vector([0., 0., 1.])

window = rectangle(1.9, 5.8)

windows = []

all_openings = []
all_closings = []
all_heats = []
def create_window(i1, i2, dist = None, width = 1., height = 1.2, bottom_height = 1., close = True):
    create = False
    pts = data['points']
    for w in data['walls']:
        if (i1 in w) and (i2 in w):
            create = True
            break
    if create:
        if not dist:
            dist = 0.5 * distance(pts[i1], pts[i2])
        window = rectangle(width, height)
        op = Opening(pts[i1], pts[i2], thick, dist, bottom_height + 0.5 * height, vertical, window, close = close)
        all_openings.append(op.ex)
        if op.close:all_closings.append(op.close)

def create_door(i1, i2, dist = None, width = 0.6, height = 2.2, close = False):
    create = False
    pts = data['points']
    for w in data['walls']:
        if (i1 in w) and (i2 in w):
            create = True
            break
    if create:
        if not dist:
            dist = 0.5 * distance(pts[i1], pts[i2])
        window = rectangle(width, height)
        op = Opening(pts[i1], pts[i2], thick, dist, 0.05 + 0.5 * height, vertical, window, close = close)
        all_openings.append(op.ex)
        if op.close:all_closings.append(op.close)

def create_heat(i1, i2, dist = None, width = 0.8, height = 0.6, offset = 0.05, box_thickness = 0.05, bottom_height = 0.2):
    create = False
    pts = data['points']
    for w in data['walls']:
        if (i1 in w) and (i2 in w):
            create = True
            break
    if create:
        if not dist:
            dist = 0.5 * distance(pts[i1], pts[i2])
        heat = rectangle(width, height)
        op = BoxOnWall( pts[i1], pts[i2], thick, box_thickness, offset, dist, 0.2 + 0.5 * height, vertical, heat)
        all_heats.append(op.ex)

p1 = Point([-25., -25., -1.5*total_height])
p2 = Point([25., 25., 1.5*total_height])
bbox = big_box(p1, p2)
create_door(26,0,width = 1.5)
create_door(13,7, width=1.5)
create_door(13,16, width=2.7, close = True)
create_door(3,7, dist = 2.)
create_door(19,3)
create_door(21,14, width = 1.2, close =True)
create_door(23,26)
create_door(19,13, close = True)
create_door(2,1)
create_door(12,11, width = 1.2, close = True)
create_door(0,6)
create_door(12,3, dist=0.5)
create_door(25,8,dist=0.5)
create_door(22,20, width = 2., close = True)
create_door(8,25, dist = 0.5)
create_door(24,23, dist = 0.8)


create_window(14,18)
create_window(17,16)
create_window(15,8)
create_window(8,10)
create_window(10,9)


create_heat(11,17)
create_heat(22,15)
create_heat(23,24)
walls_with_holes = general_split_algorithm(shapes, all_openings)
union_closings = general_fuse_algorithm(all_closings)
#union_heats = general_fuse_algorithm(all_heats)
show_shapes([[walls_with_holes, 'BLUE'],[union_closings,'RED']])
union = general_fuse_algorithm([walls_with_holes] + [union_closings] + all_heats)
volume = general_split_algorithm([bbox],[union, floor, ceiling])
#openings_union = boolean_union(all_openings)
#show_shapes([union]+[floor]+[ceiling]+[bbox])
sa = shape_array(volume)
#show_shapes([[sa, 'BLUE']]+[[walls_with_holes,'ORANGE']]+[[union_closings,'CYAN']+[[floor,'GREEN']]])
#show_shapes([[sa, 'BLUE']])

house = {'walls': [walls_with_holes], 'windows': all_closings, 'volume': sa, 'heats': all_heats}


print '-------------- INTERIOR VOLUME OK ------------------'
from OCCUtils.Topology import Topo, dumpTopology

#from OCC.Core.gp import gp_Pnt2d, gp_Pln, gp_Origin, gp_DZ, gp_Pnt, gp_Vec, gp_Dir
#topo = Topo(volume)


#dumpTopology(volume)
from OCC.Core.BRep import BRep_Tool
from OCC.Core.TopAbs import (TopAbs_VERTEX, TopAbs_EDGE, TopAbs_FACE, TopAbs_WIRE,
                        TopAbs_SHELL, TopAbs_SOLID, TopAbs_COMPOUND,
                        TopAbs_COMPSOLID)

from OCC.Core.TopoDS import TopoDS_Iterator
from OCC.TopoDS import (topods, TopoDS_Wire, TopoDS_Vertex, TopoDS_Edge,
                        TopoDS_Face, TopoDS_Shell, TopoDS_Solid,
			TopoDS_Compound, TopoDS_CompSolid, topods_Edge,
			topods_Vertex, TopoDS_Iterator)

def ident_boundaries(house):
    from dactylos.cad_functions import is_on_solid 
    appended = []
    for i,f in enumerate(house['faces']):
        # f is an array of pts
        for solid in house['windows']:
            if is_on_solid(f, solid):
	        house['boundaries']['windows'].append(i)
                appended.append(i)
        for solid in house['walls']:
            if is_on_solid(f, solid) and (i not in appended):
	        house['boundaries']['walls'].append(i)
                appended.append(i)
        for solid in house['heats']:
            if is_on_solid(f, solid) and (i not in appended):
	        house['boundaries']['heats'].append(i)
                appended.append(i)
		
	
        
    
    


def corners(house):
    d = {}
    iface = -1
    topo = Topo(house['volume'])
    house['boundaries'] = {'walls': [], 'windows': [], 'heats': []}
    house['faces'] = []
    for f in topo.wires() :
        iface += 1
        key = 'face'+str(iface)
	print 'new_face'
	edges = []
	d[key] = []
        topof = Topo(f)
        for e in topof.edges():
	    vts = Topo(e)
	    print 'edge'
	    edge = []
	    for i,v in enumerate(vts.vertices()):
                brt = BRep_Tool()
	        pnt = brt.Pnt(topods_Vertex(v))
		edge.append([pnt.X(), pnt.Y(), pnt.Z()])
            edges.append(edge)
	print len(edges)
        first_edge = edges.pop(0)
	point_array = [first_edge[0], first_edge[1]]
	print 'edges   :'
	for e in edges:print e
	print '-------'
        while len(edges)>0:
	    for i,e in enumerate(edges):
	        if point_array[-1] in e:
		    ed = edges.pop(i)
		    print point_array[-1]
		    if ed[0]==point_array[-1]:point_array.append(ed[1])
		    elif ed[1]==point_array[-1]:point_array.append(ed[0])
		    break 
        d[key] = point_array
	house['faces'].append(point_array)
    return d





	    


def dumpTopology0(shape):
    brt = BRep_Tool()
    s = shape.ShapeType()
    #if s == TopAbs_VERTEX:
    #    pnt = brt.Pnt(topods_Vertex(shape))
    #    print(".." * level  + "<Vertex %i: %s %s %s>" % (hash(shape), pnt.X(), pnt.Y(), pnt.Z()))
    #else:
    #    print(".." * level, shapeTypeString(shape))
    st+=get_dicostring(s, iface, brt, shape)
    it = TopoDS_Iterator(shape)
    while it.More():
        shp = it.Value()
        it.Next()
        dumpTopology(shp)

corners(house)
ident_boundaries(house)


import pygmsh as pg
from youbastard.geometry.Point import Point
from youbastard.io_functions import write_geo
geom = pg.built_in.Geometry()

windows_phys = []

for iface in house['boundaries']['windows']:
    lloop = []
    pts = []
    face = house['faces'][iface]
    for i,pt in enumerate(face[:-1]):
        p0 = geom.add_point((pt[0], pt[1], pt[2]), 1.)
	pts.append(p0)
    for i,p in enumerate(pts[:-1]):
	li = geom.add_line(p, pts[i+1])
	lloop.append(li)
    li = geom.add_line(pts[-1], pts[0])
    lloop.append(li)
    ll = geom.add_line_loop(lloop)
    sf = geom.add_plane_surface(ll)
    windows_phys.append(sf)

#for sf in windows_phys:
geom.add_physical_surface(windows_phys, label='windows')

wall_phys = []

for iface in house['boundaries']['walls']:
    lloop = []
    pts = []
    face = house['faces'][iface]
    for i,pt in enumerate(face[:-1]):
        p0 = geom.add_point((pt[0], pt[1], pt[2]), 1.)
	pts.append(p0)
    for i,p in enumerate(pts[:-1]):
	li = geom.add_line(p, pts[i+1])
	lloop.append(li)
    li = geom.add_line(pts[-1], pts[0])
    lloop.append(li)
    ll = geom.add_line_loop(lloop)
    sf = geom.add_plane_surface(ll)
    wall_phys.append(sf)
    
geom.add_physical_surface(wall_phys, label='walls')


heat_phys = []
for iface in house['boundaries']['heats']:
    lloop = []
    pts = []
    face = house['faces'][iface]
    for i,pt in enumerate(face[:-1]):
        p0 = geom.add_point((pt[0], pt[1], pt[2]), 1.)
	pts.append(p0)
    for i,p in enumerate(pts[:-1]):
	li = geom.add_line(p, pts[i+1])
	lloop.append(li)
    li = geom.add_line(pts[-1], pts[0])
    lloop.append(li)
    ll = geom.add_line_loop(lloop)
    sf = geom.add_plane_surface(ll)
    heat_phys.append(sf)
    
geom.add_physical_surface(heat_phys, label='heats')
name = 'maison'

write_geo(name, geom)
import subprocess
from youbastard.geometry.Triangulation import Triangulation, read_msh_file
exe_gmsh = '/home/fon/gmsh-3.0.3-git-Linux/bin/gmsh'
subprocess.call(
        [exe_gmsh, name+'.geo', '-2', '-o', 
        name+'.msh', '>', name+'.mshlog'])
tri = read_msh_file(name)

print tri['faces']['windows']

tri.write_fms_file(name)

print house.keys()
d = {}
d['faces'] = house['faces']
d['boundaries'] = house['boundaries']

json.dump(d,open('test0.json','w'),indent = 4)

#dumpTopology(sa)
