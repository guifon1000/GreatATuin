from OCC.Core.gp import gp_Pnt2d, gp_Pln, gp_Origin, gp_DZ, gp_Pnt, gp_Vec, gp_Dir
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire, BRepBuilderAPI_MakeFace
from OCC.Core.TopoDS import TopoDS_Solid
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakePrism, BRepPrimAPI_MakeBox
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCC.Core.BRepClass3d import BRepClass3d_SolidClassifier, BRepClass3d_SClassifier, BRepClass3d_SolidExplorer
from youbastard.geometry.Point import Point
from youbastard.geometry.Vector import Vector
from youbastard.geometry.Frame import Frame
from youbastard.geometry.Plane import Plane

from OCC.Core.TopoDS import TopoDS_Iterator
def show_shapes(array):
    from OCC.Display.SimpleGui import init_display
    display, start_display, add_menu, add_function_to_menu = init_display()
    for shape in array:
        ais_shape = display.DisplayColoredShape(shape[0], color=shape[1])
        display.Context.SetTransparency(ais_shape,0.5)
        #display.DisplayShape(ais_shape, update=True)
    start_display()

def shape_array(a_shape):
    iterator = TopoDS_Iterator(a_shape)
    out_shape = None
    while iterator.More():
        print '---------- iterator ------'
        print iterator.Value().Reversed().Closed()
        classifier = BRepClass3d_SolidClassifier(iterator.Value(), gp_Pnt(15.,2.,1.), 0.000001)
        test = classifier.State()
        if (test == 0) and (not out_shape):
            out_shape = iterator.Value()
        iterator.Next()
    return out_shape


def is_on_solid(pts, shape):
    # are all of the pts points on shape ?
    iterator = TopoDS_Iterator(shape)
    belong = [False for p in pts]
    for i,p in enumerate(pts):
        classifier = BRepClass3d_SolidClassifier(shape, gp_Pnt(p[0], p[1], p[2]),0.00001)
        test = classifier.State()
        if (test == 2) :
            belong[i] = True
    if belong == [True for p in pts]:
        return True
    else:
        return False 
	
def named_cartesian_box(xmin = -10., xmax = 10., ymin= -10., ymax = 10., zmin = -10., zmax=10.):
    from youbastard.geometry.Polyline3D import Polyline3D
    from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Sewing, BRepBuilderAPI_MakeSolid
    from OCC.Core.TopoDS import topods
    from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
    p0 = Point((xmin, ymin, zmin)) 
    p1 = Point((xmin, ymax, zmin)) 
    p2 = Point((xmax, ymax, zmin)) 
    p3 = Point((xmax, ymin, zmin))
    p4 = Point((xmin, ymin, zmax)) 
    p5 = Point((xmin, ymax, zmax)) 
    p6 = Point((xmax, ymax, zmax)) 
    p7 = Point((xmax, ymin, zmax))
    
    pol_Xmin = Polyline3D([p0, p1, p5, p4, p0][::-1])
    pol_Xmax = Polyline3D([p2, p3, p7, p6, p2][::-1])
    pol_Ymin = Polyline3D([p1, p2, p6, p5, p1][::-1])
    pol_Ymax = Polyline3D([p0, p4, p7, p3, p0][::-1])
    pol_Zmin = Polyline3D([p0, p3, p2, p1, p0][::-1])
    pol_Zmax = Polyline3D([p5, p6, p7, p4, p5][::-1])

    dicoface = {
             'Xmin': face_polyline3d(pol_Xmin).Shape(),
             'Xmax': face_polyline3d(pol_Xmax).Shape(),
             'Ymin': face_polyline3d(pol_Ymin).Shape(),
             'Ymax': face_polyline3d(pol_Ymax).Shape(),
             'Zmin': face_polyline3d(pol_Zmin).Shape(),
             'Zmax': face_polyline3d(pol_Zmax).Shape()
    }
    sew = BRepBuilderAPI_Sewing()

    make_solid = BRepBuilderAPI_MakeSolid()
    for k in dicoface.keys():
        sew.Add(dicoface[k])
    sew.Perform()
    shell = sew.SewedShape()
    make_solid.Add(topods.Shell(shell))
    box = make_solid.Solid()
    box = BRepPrimAPI_MakeBox(gp_Pnt(xmin,ymin,zmin), gp_Pnt(xmax,ymax,zmax))
    return box, dicoface


class Part(object):
    def __init__(self, name, dict_of_shells, solid):
        from OCC.Core.Bnd import Bnd_Box
        from OCC.Core.BRepBndLib import brepbndlib_Add
        from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
        bbox = Bnd_Box()
        bbox.SetGap(1.e-6)
        self.name = name
        self.solid = solid
        self.dico = {}
        self.dicosplit = {}
        self.kept = []
        self.to_check = []
        self.splitsolid = []
        brepbndlib_Add(solid, bbox, False)
        xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
        self.box = BRepPrimAPI_MakeBox(gp_Pnt(xmin,ymin,zmin), gp_Pnt(xmax,ymax,zmax))
        for k in dict_of_shells:
            new_k = name + k
            self.dico[k] = dict_of_shells[k]
            self.dicosplit[k] = None


def debox(box, array):
    from OCC.Core.BRepClass3d import BRepClass3d_SolidClassifier
    from OCC.Core.TopoDS import topods_Vertex
    from OCC.Extend.TopologyUtils import TopologyExplorer
    out = []
    for shp in [pt.solid for pt in array]:
        tool = [shp]
        arg = [box]
        tmp = general_split_algorithm(arg, tool)
        box = unify_same_domain(tmp)
        for s in TopologyExplorer(box).solids():
            classifier = BRepClass3d_SolidClassifier(box, gp_Pnt(-0.5, 0.5, 0.5),1.e-12)
            if classifier.State() == 0:
                print 'found'
                box = s
                break
    print '------------------'
    return TopologyExplorer(box).solids()




def cut_shells_by_box(cutbox, dico):
    nd = {}
    for k in dico:
        arg = [dico[k]]
        tool = [cutbox.Shape()]
        nd[k] = general_split_algorithm(arg, tool)
    return nd


def solid_hole_in_box(box, dicobox, solid):
    from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section, BRepAlgoAPI_Cut
    from OCC.Core.BRepFeat import BRepFeat_SplitShape
    from OCC.TopTools import TopTools_ListIteratorOfListOfShape
    from OCC.Extend.TopologyUtils import TopologyExplorer
    from OCC.Core.TopoDS import TopoDS_Face
    #array contains several parts
    d = {}
    #for solid in [p.solid for p in array]:
    #d[ki] = []
    
    for k in dicobox:
        arg = [dicobox[k]]
        tool = [solid]
        d[k] = general_split_algorithm(arg, tool)
    return d


def unify_same_domain(solid):
    from OCC.Core.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
    up = ShapeUpgrade_UnifySameDomain(solid)
    return up.Shape()



def merge_all_solids(array):
    return general_fuse_algorithm([s.solid for s in array])


def cut_all_solids(array):
    out = []
    for i,si in enumerate(array):
        arg = [si.solid]
        print '#############"'
        print 'solid '+str(i)
        tools = [array[j].solid for j in range(len(array)) if j!=i]
        out.append(general_split_algorithm(arg, tools))
    return out


def common_shells_with_solids(array_of_parts, array):
    from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Common
    from OCC.Extend.TopologyUtils import TopologyExplorer
    d = {}
    for i,si in enumerate(array_of_parts):
        for k in si.dico:
            named_shell = si.dico[k]
            for j,s in enumerate(array):
                com = BRepAlgoAPI_Common(named_shell.Shape(), s)
                n_common_faces = TopologyExplorer(com.Shape()).number_of_faces()
                if n_common_faces >0:
                    d[k] = com.Shape()
                    break
    return d





def cut_solid_parts(array):
    from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section
    from OCC.Core.BRepFeat import BRepFeat_SplitShape
    from OCC.TopTools import TopTools_ListIteratorOfListOfShape
    from OCC.Extend.TopologyUtils import TopologyExplorer
    from OCC.Core.TopoDS import TopoDS_Face
    #array contains several parts
    for i, si in enumerate(array):
        #for ki in si.dico:
        shpi = si.solid
        split = BRepFeat_SplitShape(shpi)
        for j,sj in enumerate(array):
            if i!=j:
                shpj = sj.solid
                sect = BRepAlgoAPI_Section(shpi, shpj , False)
                sect.ComputePCurveOn1(True)
                sect.Approximation(True)
                sect.SetFuzzyValue(1.e-12)
                sect.Build()
                shpsect = sect.Shape()
                for edg in TopologyExplorer(shpsect).edges():
                    face = TopoDS_Face()
                    if sect.HasAncestorFaceOn1(edg, face) :
                        split.Add(edg, face)
        split.Build()
        lst = TopTools_ListIteratorOfListOfShape(split.Modified(shpi))
        while lst.More():
            for face in TopologyExplorer(lst.Value()).faces():
                array[i].splitsolid.append(face)
            lst.Next()


def common_parts(array, shells):
    from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Common
    from OCC.Extend.TopologyUtils import TopologyExplorer
    d = {}
    for i,si in enumerate(array):
        for k in si.dico:
            named_shell = si.dico[k]
            for j,s in enumerate(shells):
                com = BRepAlgoAPI_Common(named_shell.Shape(), s)
                n_common_faces = TopologyExplorer(com.Shape()).number_of_faces()
                if n_common_faces >0:
                    d[k] = com.Shape()
                    break
    return d



def split_shells_by_solid(array, solid, display):
    from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section
    from OCC.Core.BRepFeat import BRepFeat_SplitShape
    from OCC.TopTools import TopTools_ListIteratorOfListOfShape
    from OCC.Extend.TopologyUtils import TopologyExplorer
    from OCC.Core.TopoDS import TopoDS_Face
    for i, si in enumerate(array):
        for ki in si.dico:
            print ki
            sfi = si.dico[ki]
            split = BRepFeat_SplitShape(sfi.Shape())
            sect = BRepAlgoAPI_Section(solid, sfi.Shape() , False)
            sect.ComputePCurveOn1(True)
            sect.Approximation(True)
            sect.SetFuzzyValue(1.e-12)
            sect.Build()
            shpsect = sect.Shape()
            for edg in TopologyExplorer(shpsect).edges():
                face = TopoDS_Face()
                if sect.HasAncestorFaceOn1(edg, face) :
                    split.Add(edg, face)
            split.Build()
            ais_shape = display.DisplayColoredShape(split.Shape(), color='RED')
            display.Context.SetTransparency(ais_shape,0.5)






def parts_in_others_boxes(array):
    from OCC.Extend.TopologyUtils import TopologyExplorer
    from OCC.Core.BRepClass3d import BRepClass3d_SolidClassifier
    from OCC.Core.TopoDS import topods_Vertex
    from OCC.Core.BRep import BRep_Tool
    for i, si in enumerate(array):
        for faci in si.splitsolid:
            for j,sj in enumerate(array):
                if i!=j:
                    boxj = sj.box.Shape()
                    states = []
                    for v in TopologyExplorer(faci).vertices():
                        brt = BRep_Tool()
                        pnt = brt.Pnt(topods_Vertex(v))
                        classifier = BRepClass3d_SolidClassifier(boxj, gp_Pnt(pnt.X(), pnt.Y(), pnt.Z()),1.e-12)
                        states.append(classifier.State())
                    if 0 in states:
                        si.to_check.append([faci,j])
            if faci not in [ftc[0] for ftc in si.to_check] :
                si.kept.append(faci)





def reconstruct_parts(array):
    shells = []
    from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Sewing
    for i,si in enumerate(array):
        shell = BRepBuilderAPI_Sewing()
        for face in si.kept:
            shell.Add(face)
        shell.Perform()
        shells.append(shell.SewedShape())
    return shells

def reconstruct_object(array):
    from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Sewing, BRepBuilderAPI_MakeSolid
    from OCC.Core.BRepGProp import brepgprop_VolumeProperties
    from OCC.Core.GProp import GProp_GProps
    from OCC.Core.TopoDS import topods
    obj = BRepBuilderAPI_Sewing()
    for i,si in enumerate(array):
        obj.Add(si)
    obj.Perform()
    shell = obj.SewedShape()
    make_solid = BRepBuilderAPI_MakeSolid()
    make_solid.Add(topods.Shell(shell))
    gp = GProp_GProps()

    #make_solid.Add(shell)
    solid = make_solid.Solid()
    brepgprop_VolumeProperties(shell,gp)
    print 'RECONSTRUCTED SOLID VOLUME :'
    print gp.Mass()
    return solid

def check_remaining(array):
    from OCC.Extend.TopologyUtils import TopologyExplorer
    from OCC.Core.BRepClass3d import BRepClass3d_SolidClassifier
    from OCC.Core.TopoDS import topods_Vertex
    from OCC.Core.BRep import BRep_Tool
    for i, si in enumerate(array):
        while si.to_check:
            face, isol = si.to_check.pop(0)
            brt = BRep_Tool()
            states = []
            for v in TopologyExplorer(face).vertices():
                pnt = brt.Pnt(topods_Vertex(v))
                classifier = BRepClass3d_SolidClassifier(array[isol].solid, gp_Pnt(pnt.X(), pnt.Y(), pnt.Z()),1.e-10)
                states.append(classifier.State())
            if (0 not in states)  :
                #if 'fore' in array[isol].name: print states
                si.kept.append(face)


def extrusion_to_solid(extrusion, name):
    from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Sewing, BRepBuilderAPI_MakeSolid
    from OCC.Core.TopoDS import topods
    from OCC.Core.BRepGProp import brepgprop_VolumeProperties
    from OCC.Core.GProp import GProp_GProps
    from OCCUtils.Topology import Topo
    d = extrusion_to_ruled_surfaces(extrusion, cap = True)
    sew = BRepBuilderAPI_Sewing()

    make_solid = BRepBuilderAPI_MakeSolid()
    faces = {}
    for k in d.keys():
        faces[name+'_'+k] = d[k]
        sew.Add(d[k].Shape())
    sew.Perform()
    shell = sew.SewedShape()
    make_solid.Add(topods.Shell(shell))
    gp = GProp_GProps()

    #make_solid.Add(shell)
    solid = make_solid.Solid()
    brepgprop_VolumeProperties(shell,gp)
    print gp.Mass()
        
    return Part(name, faces, solid)
    #return solid, faces
    

def extrusion_to_ruled_surfaces(extrusion, cap = True):
    from OCC.Core.BRepOffsetAPI import BRepOffsetAPI_ThruSections
    from youbastard.geometry.Extrusion import Extrusion
    from youbastard.geo_functions import assembly_polylines3d
    #from OCC.Display.SimpleGui import init_display
    #display, start_display, add_menu, add_function_to_menu = init_display()
    dico = {}
    for k in extrusion.keys():
        array_of_polylines = extrusion[k]
        generator = BRepOffsetAPI_ThruSections(False, True)
        for pol in array_of_polylines:
            wire = wire_from_polyline(pol)
            generator.AddWire(wire.Wire())
            #dico[k].append(wire.Shape())
        generator.Build()
        dico[k] = generator
    if cap:
        first_cap_polylines = []
        last_cap_polylines = []
        for k in extrusion.keys():
            first_cap_polylines.append(extrusion[k][0])
            last_cap_polylines.append(extrusion[k][-1])
        ordered_first = assembly_polylines3d(first_cap_polylines) 
        ordered_last = assembly_polylines3d(last_cap_polylines)
	print ordered_first
        dico['first_tip'] = face_polyline3d(ordered_first) 
        dico['last_tip'] = face_polyline3d(ordered_last)
    return dico


def polyline3d_to_edges(pol3d):
    lines= []
    print pol3d
    for i,p in enumerate(pol3d[:-1]):
        gp0 = gp_Pnt(p[0], p[1], p[2])
        gp1 = gp_Pnt(pol3d[i+1][0] , pol3d[i+1][1], pol3d[i+1][2])
        lines.append(
                BRepBuilderAPI_MakeEdge(gp0, gp1).Edge()
                )
    return lines

    
def face_polyline3d(pol3d):
    from OCC.Core.TopoDS import topods
    lines= []
    for i,p in enumerate(pol3d[:-1]):
        gp0 = gp_Pnt(p[0], p[1], p[2])
        gp1 = gp_Pnt(pol3d[i+1][0] , pol3d[i+1][1], pol3d[i+1][2])
        lines.append(
                BRepBuilderAPI_MakeEdge(gp0, gp1).Edge()
                )
    wire = BRepBuilderAPI_MakeWire(lines[0])
    for l in lines[1:]:
        wire.Add(l)
    face = BRepBuilderAPI_MakeFace(wire.Wire())
    return face


def face_polyline(polyline, frame):
    pol3d = polyline.to_frame(frame)
    lines = []
    yb_point = Point([frame[0][i] for i in range(3)])
    yb_vec = Vector([frame[3][i] for i in range(3)]).unit()
    orig = gp_Pnt(frame[0][0], frame[0][1], frame[0][2])
    vec = gp_Dir(yb_vec[0], yb_vec[1], yb_vec[2])
    plane = gp_Pln(orig, vec)

    for i,p in enumerate(pol3d[:-1]):
        gp0 = gp_Pnt(p[0], p[1], p[2])
        gp1 = gp_Pnt(pol3d[i+1][0] , pol3d[i+1][1], pol3d[i+1][2])
        lines.append(
                BRepBuilderAPI_MakeEdge(gp0, gp1).Edge()
                )

    wire = BRepBuilderAPI_MakeWire(lines[0])

    for l in lines[1:]:
        wire.Add(l)

    face = BRepBuilderAPI_MakeFace(wire.Wire())
    return face.Shape()


def wire_from_polyline(pol3d):
    lines = []
    for i,p in enumerate(pol3d[:-1]):
        gp0 = gp_Pnt(p[0], p[1], p[2])
        gp1 = gp_Pnt(pol3d[i+1][0] , pol3d[i+1][1], pol3d[i+1][2])
        lines.append(
                BRepBuilderAPI_MakeEdge(gp0, gp1).Edge()
                )
    wire = BRepBuilderAPI_MakeWire(lines[0])

    for l in lines[1:]:
        wire.Add(l)
    return wire


def spline_from_polyline(pol):
    from OCC.Extend.ShapeFactory import point_list_to_TColgp_Array1OfPnt, make_face
    from OCC.Core.GeomAPI import GeomAPI_PointsToBSpline
    array = []
    for p in pol:
        array.append(gp_Pnt(p[0], p[1], p[2]))
    pt_list1 = point_list_to_TColgp_Array1OfPnt(array)
    SPL1 = GeomAPI_PointsToBSpline(pt_list1).Curve()
    SPL1 = GeomAPI_PointsToBSpline(pt_list1)
    edge = BRepBuilderAPI_MakeEdge(SPL1.Curve())
    wire = BRepBuilderAPI_MakeWire(edge.Edge())
    return edge.Shape(), wire

    


def extrude_polyline2d(polyline, frame, height):
    pol3d = polyline.to_frame(frame)
    lines = []
    yb_point = Point([frame[0][i] for i in range(3)])
    yb_vec = Vector([frame[1][0][i] for i in range(3)]).unit()
    print '*************'
    print yb_vec
    orig = gp_Pnt(frame[0][0], frame[0][1], frame[0][2])
    vec = gp_Dir(yb_vec[0], yb_vec[1], yb_vec[2])
    plane = gp_Pln(orig, vec)

    for i,p in enumerate(pol3d[:-1]):
        print p
        print 'zob'
        gp0 = gp_Pnt(p[0], p[1], p[2])
        gp1 = gp_Pnt(pol3d[i+1][0] , pol3d[i+1][1], pol3d[i+1][2])
        lines.append(
                BRepBuilderAPI_MakeEdge(gp0, gp1).Edge()
                )

    wire = BRepBuilderAPI_MakeWire(lines[0])

    for l in lines[1:]:
        wire.Add(l)

    face = BRepBuilderAPI_MakeFace(wire.Wire())
    print 'normal'
    print [vec.X(), vec.Y(), vec.Z() ]
    extrude = BRepPrimAPI_MakePrism(face.Shape(), gp_Vec(height * vec.X(), height * vec.Y(), height * vec.Z())).Shape()
    return extrude

def big_box(pt1, pt2):
    gp1 = gp_Pnt(pt1[0], pt1[1], pt1[2])
    gp2 = gp_Pnt(pt2[0], pt2[1], pt2[2])
    gpm = gp_Pnt(0.5*(pt1[0] + pt2[0]), 
                 0.5*(pt1[1] + pt2[1]), 
                 0.5*(pt1[2] + pt2[2])) 
    box = BRepPrimAPI_MakeBox(gp1, gp2)
    return box.Shape()

def general_fuse_algorithm(array):
    from OCC.Core.BOPAlgo import BOPAlgo_Builder
    builder = BOPAlgo_Builder()
    for s in array:
        builder.AddArgument(s)
    builder.Perform()
    result = builder.Shape()
    return result


def general_split_algorithm(array, tool):
    from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCC.Core.TopTools import TopTools_ListOfShape
    cut = BRepAlgoAPI_Cut()
    L1 = TopTools_ListOfShape()
    for s in array:
        L1.Append(s)
    L2 = TopTools_ListOfShape()
    for t in tool:
        L2.Append(t)
    cut.SetArguments(L1)
    cut.SetTools(L2)
    cut.SetFuzzyValue(1.e-18)
    cut.SetRunParallel(False)
    cut.Build()
    return cut.Shape()






def boolean_union(array):
    first = array[0]
    if len(array)>1:
        for i in range(1, len(array)):
            other = array[i]
            fuse = BRepAlgoAPI_Fuse(first, other)
            first = fuse.Shape()
    return [first]
    
