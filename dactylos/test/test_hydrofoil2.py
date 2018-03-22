from youbastard.analytic_functions.bezier_functions import chord_function
from youbastard.geo_functions import distance
from youbastard.modelers.profiles.splineProfileMultiParam import Profile
from youbastard.geometry.Vector import Vector
from youbastard.geometry.Point import Point
from youbastard.geometry.Polyline2D import Polyline2D
from youbastard.geometry.Spline3D import Spline3D
from youbastard.geometry.Extrusion import Extrusion
from youbastard.geometry.Segment import Segment
from dactylos.cad_functions import (extrusion_to_ruled_surfaces, big_box, 
                                   general_fuse_algorithm, general_split_algorithm, 
                                   extrusion_to_solid, named_cartesian_box, Part, cut_solid_parts,
                                   parts_in_others_boxes, check_remaining, reconstruct_parts,
                                   reconstruct_object)
from OCC.Display.SimpleGui import init_display
import pygmsh as pg
from youbastard.io_functions import write_geo

geom = pg.built_in.Geometry()

display, start_display, add_menu, add_function_to_menu = init_display()
def lin_fun(value1, value2):
    f = lambda x: value1 + (value2 - value1) * x
    return f


ndisc = 80
fore_span = 0.5
aft_span = 0.15
fore_max_chord = 0.12
aft_max_chord = 0.06
fore_chordfun = chord_function(0.78)
aft_chordfun = chord_function(0.9)
fore_chordf = lambda s: fore_max_chord * fore_chordfun(s)
aft_chordf = lambda s: aft_max_chord * aft_chordfun(s)
fore_rot_fun = lin_fun(0.01, 0.2)
aft_rot_fun = lin_fun(0.005, 0.12)
fore_profile = Profile(typ = 'fon',par = [0.82  ,0.15 , 0.02 , 0.05 , 0.015],npt = 7) # creation of the 2d profile
aft_profile = Profile(typ = 'fon',par = [0.69  ,0.098 , 0.012 , 0.017 , 0.0025],npt = 5) # creation of the 2d profile
fin_profile = Profile(typ = 'fon',par = [0.89  ,0.08 , 0.0082 , 0. , 0.],npt = 5) # creation of the 2d profile
fore_pol = fore_profile.polyline(closed = True)
aft_pol = aft_profile.polyline(closed = True)
fin_pol = fin_profile.polyline(closed = True)

fore_gen_point = [
        Point([0., 0. ,0.]),
        Point([0.0,fore_span/4,0.002]),
        Point([0.,fore_span/2,0.01]),
        Point([-0.02,2.*fore_span/3.,0.015]),
        Point([-0.08,fore_span,-0.02])
            ]

deltaX = -0.6

aft_gen_point = [
        Point([deltaX ,0. ,0.]),
        Point([deltaX , aft_span/4,-0.002]),
        Point([deltaX ,aft_span/2,-0.005]),
        Point([deltaX - 0.002,2.*aft_span/3.,-0.008]),
        Point([deltaX-0.008,aft_span,0.006])
            ]


fore_generator = Spline3D(fore_gen_point)
aft_generator = Spline3D(aft_gen_point)


if fore_pol.is_closed:
    open_fore_pol = fore_pol[:-1]
else:
    open_fore_pol = fore_pol
fore_npt = len(open_fore_pol)


if aft_pol.is_closed:
    open_aft_pol = aft_pol[:-1]
else:
    open_aft_pol = aft_pol
aft_npt = len(open_aft_pol)

aft_pol = {
        'extrados': Polyline2D(open_aft_pol[:(aft_npt-1)/2+1]),
        'intrados': Polyline2D(open_aft_pol[(aft_npt-1)/2:]),
        'trailing_edge': Polyline2D([open_aft_pol[-1], open_aft_pol[0]])
        }

fore_pol = {
        'extrados': Polyline2D(open_fore_pol[:(fore_npt-1)/2+1]),
        'intrados': Polyline2D(open_fore_pol[(fore_npt-1)/2:]),
        'trailing_edge': Polyline2D([open_fore_pol[-1], open_fore_pol[0]])
        }

aft_wing = Extrusion(aft_pol, aft_generator, scale = aft_chordf, rotate = aft_rot_fun, n = 5)
fore_wing = Extrusion(fore_pol, fore_generator, scale = fore_chordf, rotate = fore_rot_fun, n=7)

from youbastard.geometry.Polyline2D import Circle
from youbastard.geometry.ParametricCurve3D import ParametricCurve3D
from youbastard.analytic_functions.bezier_functions import piecewise_bezier_polyline

cir = Circle(nseg = 50)


cir_pol = {'surface': Polyline2D(cir)}
fin_pol = {'surface': Polyline2D(fin_pol[::-1])}


fun_scale = piecewise_bezier_polyline(0.2,0.1,[[0.025,0.65,0.5],[0.1,1.,0.5], [0.3,0.9,0.5],[0.4,0.7,0.5],[0.5,0.5,0.5]] )
funR = lambda s: 0.015 * fun_scale(s)
funX = lambda s: 0.05 -  0.75 * s
funY = lambda s: 0.
funZ = lambda s: 0.
med_rot = lambda s: 0.
med_gen = ParametricCurve3D(funX, funY, funZ)

medax = Extrusion(cir_pol, med_gen, scale = funR, rotate = med_rot, n = 90)


fin_scale = lambda s: 0.08
finR = lambda s: 0.
finX = lambda s: deltaX/2.
finY = lambda s: 0.
finZ = lambda s: 0.9 * (1. -  s)
fin_rot = lambda s: 0.

fin_gen = ParametricCurve3D(finX, finY, finZ)
fin = Extrusion(fin_pol, fin_gen, mode = Vector([0.,1.,0.]), scale = fin_scale, rotate = fin_rot, n=13)
print fin

medax.pop_to_geom(geom)
aft_wing.pop_to_geom(geom)
fore_wing.pop_to_geom(geom)
fin.pop_to_geom(geom)
write_geo('medax', geom)

import sys
#fore_cad = extrusion_to_ruled_surfaces(fore_wing, cap = True)
#aft_cad = extrusion_to_ruled_surfaces(aft_wing, cap = True)
wing1 = extrusion_to_solid(fore_wing, 'fore')
wing2 = extrusion_to_solid(aft_wing, 'aft')
med_ax = extrusion_to_solid(medax, 'med')
fin_s = extrusion_to_solid(fin, 'fin')
#bb, boundaries = named_cartesian_box(xmin = -10., xmax = 10. , ymin = 0., ymax = 10., zmin=-5., zmax=5.)
def merge_dicts(*largs):
    d= {}
    for dico in largs[0]:
        for key in dico:
	    if not d.has_key(key):d[key] = dico[key]
    return d

all_parts = [wing1, wing2, med_ax, fin_s]

from dactylos.cad_functions import cut_all_solids, common_shells_with_solids, merge_all_solids, unify_same_domain, debox

#merged = merge_all_solids(all_parts)

unified = []

box, dicobox = named_cartesian_box(xmin = -1., xmax = 0., ymin= 0., ymax = 1., zmin = -1., zmax=0.5)
#for s in [pt.solid for pt in all_parts]:
#    unified.append(unify_same_domain(s))

#cut_parts = cut_all_solids(all_parts)
#dcommon = common_shells_with_solids(all_parts, cut_parts)
unified = debox(box.Shape(), all_parts)

i=0
for k in unified:
    #print k, switch_colors(i)
    #ais_shape = display.DisplayColoredShape(ss[k][0], color='RED')
    ais_shape = display.DisplayColoredShape(k, color='GREEN')
    display.Context.SetTransparency(ais_shape,0.8)
    #i+=1


start_display()
sys.exit()


cut_solid_parts(all_parts)

parts_in_others_boxes(all_parts)
check_remaining(all_parts)
shells = reconstruct_parts(all_parts)
foil = reconstruct_object(shells)






print '***********'
from dactylos.cad_functions import split_shells_by_solid, common_parts, named_cartesian_box, solid_hole_in_box, general_split_algorithm, cut_shells_by_box
d = common_parts(all_parts, shells)

box, dicobox = named_cartesian_box(xmin = -1., xmax = 1., ymin= 0., ymax = 1., zmin = -1., zmax=1.)
cutbox, dicobox2 = named_cartesian_box(xmin = -1., xmax = 1., ymin= -1., ymax = 0., zmin = -1., zmax=1.)
ss = solid_hole_in_box(cutbox.Shape(), dicobox, foil)

test = cut_shells_by_box(cutbox, d)


#ss = general_split_algorithm([box.Shape()], [foil])

def switch_colors(i):
    colors = ['BLUE', 'RED', 'GREEN', 'CYAN', 'BLACK', 'ORANGE']
    nc = len(colors)
    return colors[i%nc]

i=0
for k in ss:
    #print k, switch_colors(i)
    #ais_shape = display.DisplayColoredShape(ss[k][0], color='RED')
    ais_shape = display.DisplayColoredShape(ss[k], color='GREEN')
    display.Context.SetTransparency(ais_shape,0.8)
    #i+=1

for k in d:
    ais_shape = display.DisplayColoredShape(d[k], color='RED')


#ais_shape = display.DisplayColoredShape(ss, color='GREEN')
#display.Context.SetTransparency(ais_shape,0.8)



#split_shells_by_solid(all_parts, foil, display)


#for k in test:
#    print k
#    ais_shape = display.DisplayColoredShape(test[k], color='RED')
#    display.Context.SetTransparency(ais_shape,0.5)

#for sol in shells:
#ais_shape = display.DisplayColoredShape(foil, color='GREEN')
            #display.Context.SetTransparency(ais_shape,0.5)
        #for f in sol.to_check[k]: 
        #    ais_shape = display.DisplayColoredShape(f, color='RED')
        #    display.Context.SetTransparency(ais_shape,0.5)



display.DisplayColoredShape(wing1,'RED')
display.DisplayColoredShape(wing2,'RED')
display.DisplayColoredShape(med_ax,'RED')
surfaces = [wing1, wing2, med_ax]


from OCC.Core.BOPAlgo import BOPAlgo_MakerVolume
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCC.Core.GEOMAlgo import GEOMAlgo_Splitter
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_SHELL, TopAbs_WIRE, TopAbs_EDGE, TopAbs_VERTEX
from OCCUtils.Topology import Topo, dumpTopology, shapeTypeString
from OCC.Core.BRepClass3d import BRepClass3d_SolidClassifier
from OCC.Core.BRep import BRep_Tool
from OCC.Core.TopTools import TopTools_ListIteratorOfListOfShape, TopTools_ListOfShape, TopTools_SequenceOfShape
from OCC.Core.BOPAlgo import BOPAlgo_MakerVolume
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section
from OCC.Core.TopoDS import TopoDS_Face, TopoDS_Edge
from OCC.Core.BOPAlgo import BOPAlgo_PaveFiller
from OCC.Core.BRepFeat import BRepFeat_SplitShape
from OCC.Core.BRep import BRep_Tool
from OCC.Core.TopoDS import topods_Vertex, topods_Face
from OCC.Core.BOPTools import BOPTools_AlgoTools2D_MakePCurveOnFace 
from OCC.Core.TopOpeBRepTool import TopOpeBRepTool_CurveTool_MakePCurveOnFace 
import time


start_display()
sys.exit()
def segments_to_polyline(array, istart=0, tol=1.e-10):
    from youbastard.geometry.Polyline3D import Polyline3D
    first = array[0]
    init = len(array)
    ordered = [array.pop(istart)]
    cont = True
    while len(ordered) <= len(array):
        for i,s in enumerate(array):
            p0 = s[0]
            p1 = s[1]
            last = ordered[-1][1]
            if distance(last,p0) < tol:
                ordered.append(array.pop(i))
                continue
            if distance(last, p1) < tol:
                ordered.append(array.pop(i)[::-1])
                continue
        ordered = [s[::-1] for s in ordered[::-1]]
    if ordered[0] == first:
        ordered = ordered
    else:
        ordered = ordered[::-1]
    pol = [ordered[0][0], ordered[0][1]]
    for s in ordered[1:]:
        pol.append(s[1])
    return Polyline3D(pol)


from dactylos.cad_functions import polyline3d_to_edges
for f in faces1:
    #display.DisplayColoredShape(faces1[f],'BLUE')
    #iterator = TopTools_ListIteratorOfListOfShape(explorer)
    section = BRepAlgoAPI_Section(faces1[f].Shape(), faces_ax[faces_ax.keys()[0]].Shape(),False)
    #section.SetFuzzyValue(1.e-18)
    section.ComputePCurveOn1(True)
    section.ComputePCurveOn2(True)
    section.Approximation(True)
    section.Build()
    #dumpTopology(section.Shape())
    display.DisplayColoredShape(section.Shape(),'GREEN')
    all_points = []
    all_edges = []
    for edg in Topo(section.Shape()).edges():
        brt = BRep_Tool()
        print '--- edge -----'
        edge_points = []
        tpedg = Topo(edg)
        for v in tpedg.vertices():
            pnt = brt.Pnt(topods_Vertex(v))
            edge_points.append(Point([pnt.X(), pnt.Y(), pnt.Z()]))

            display.DisplayColoredShape(pnt,'GREEN')
        all_points.append(edge_points)
    if len(all_points)!=0 : 
        pol = segments_to_polyline(all_points)
        all_edges = polyline3d_to_edges(pol)
    for edg in all_edges:
        face_explorer = TopExp_Explorer(faces1[f].Shape(), TopAbs_FACE)
        while False:#face_explorer.More():
            try:
                TopOpeBRepTool_CurveTool_MakePCurveOnFace(edg, topods_Face(face_explorer.Current()))
                print 'done'
            except:
                print 'rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr'
            face_explorer.Next()
    print 'out'
    ss = BRepFeat_SplitShape(faces1[f].Shape())
    ss.SetCheckInterior(True)
    for edg in all_edges:
        face = TopoDS_Face()
        if section.HasAncestorFaceOn1(edg, face):
            ss.Add(edg, face)
            display.DisplayColoredShape(face,'RED')
    ss.Build()
    dl = ss.DirectLeft()
    test = TopTools_ListIteratorOfListOfShape(dl)
    #display.DisplayColoredShape(ss.Shape(),'GREEN')
    display.DisplayColoredShape(faces1[f].Shape(),'RED')
#surfaces = [med_ax]
#for k in fore_cad:
#    surfaces.append(fore_cad[k])
#    display.DisplayShape(fore_cad[k])
#for k in aft_cad:
#    surfaces.append(fore_cad[k])
#    display.DisplayShape(aft_cad[k])

#display.DisplayShape(wing1)
all_s = general_fuse_algorithm(surfaces)

print type(bb)

#display.DisplayShape(all_s)
domain = general_split_algorithm([bb], surfaces)



from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section



#display.DisplayShape(domain)




from OCC.Core.STEPControl import STEPControl_Writer, STEPControl_AsIs
step_writer = STEPControl_Writer()
step_writer.Transfer(domain, STEPControl_AsIs)

stpfile = "foil.stp"


step_writer.Write(stpfile)



topo = Topo(domain)
faces = topo.faces
iface=0
print dir(faces)
N = float(topo.number_of_faces())
surfaces = [None for i in range(topo.number_of_faces())]
maxima = [-1 for i in range(topo.number_of_faces())]

for f in topo.faces():
    print str(100. * float(iface)/N)+' % done'
    for f2 in d_faces:
        shp = BRepAlgoAPI_Section(f,d_faces[f2] ).Shape()
	tp = Topo(shp)
	nedge = tp.number_of_edges()
	if (nedge > maxima[iface]) :
	    surfaces[iface] = f2
	    maxima[iface] = nedge 
    iface+=1

gmsh_dict = {}

for i in range(topo.number_of_faces()):
    if not gmsh_dict.has_key(surfaces[i]):
        gmsh_dict[surfaces[i]] = []
    gmsh_dict[surfaces[i]].append(i+1)

print gmsh_dict
name = stpfile.replace('.stp','')
fgeo = open(stpfile.replace('stp','geo'), 'w')

fgeo.write('Merge \"'+stpfile+'\";\n')
for bc in gmsh_dict:
    li = 'Physical Surface(\"'+bc+'\") = {'
    for i,f in enumerate(gmsh_dict[bc]):
        if i==len(gmsh_dict[bc])-1:
	    sep = '};\n'
	else:
	    sep = ', '
        li+=str(f)+sep
    fgeo.write(li)
fgeo.close()
    #if touching>1:
        #display.DisplayColoredShape(f, 'RED')
    #step_writer = STEPControl_Writer()
    #step_writer.Transfer(f, STEPControl_AsIs)
    #step_writer.Write("surface_"+str(i)+".stp")

import subprocess
from youbastard.geometry.Triangulation import Triangulation, read_msh_file
exe_gmsh = '/home/fon/gmsh-3.0.3-git-Linux/bin/gmsh'
subprocess.call(
        [exe_gmsh, name+'.geo', '-2', '-o', 
        name+'.msh', '>', name+'.mshlog'])
tri = read_msh_file(name)


tri.reverse().write_fms_file(name)


