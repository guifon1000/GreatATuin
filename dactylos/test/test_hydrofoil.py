from youbastard.modelers.profiles.splineProfileMultiParam import Profile
from youbastard.geometry.Point import Point
from youbastard.geometry.Vector import Vector
from youbastard.geometry.Frame import Frame
from youbastard.geometry.Spline3D import Spline3D
from youbastard.io_functions import write_geo
from dactylos.cad_functions import wire_from_polyline, show_shapes, general_fuse_algorithm
import pygmsh as pg
from OCC.Display.SimpleGui import init_display
display, start_display, add_menu, add_function_to_menu = init_display()
from OCC.Core.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCC.Core.GeomFill import GeomFill_SectionGenerator
from OCC.Core.BRepOffsetAPI import BRepOffsetAPI_ThruSections
from OCC.Core.gp import gp_Pnt
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
from youbastard.analytic_functions.bezier_functions import chord_function
from youbastard.geo_functions import distance

geom = pg.built_in.Geometry()


ndisc = 80
span = 0.8

max_chord = 0.12
scalef = chord_function(0.78)
chordf = lambda s: max_chord * scalef(s)



a_point = [
        Point([0.,span/float(ndisc),0.]),
        Point([0.01,span/4,0.002]),
        Point([0.,span/2,0.01]),
        Point([-0.02,2.*span/3.,0.015]),
        Point([-0.08,span,-0.02])
            ]


generator = Spline3D(a_point)
fa = generator.frame_array(mode = Vector([0.,0.,1.]), n=ndisc)

#fa = [Frame([[0.,0.,0.], [[0.,0.,1.],[1.,0.,0.],[0.,1.,0.]]])] + fa

shapes = []
generator_extrados = BRepOffsetAPI_ThruSections(False, True)
generator_intrados = BRepOffsetAPI_ThruSections(False, True)
generator_trailing_edge = BRepOffsetAPI_ThruSections(False, True)
generator = BRepOffsetAPI_ThruSections(False, True)
print dir(generator)

chord = lambda x: max_chord * chordf(x)
for i,f in enumerate(fa):
    # adimensioned position along span
    xspan = distance(fa[0][0], f[0])/span

    # dimesioned chord value at xspan
    local_chord = chord(xspan)

    # the profile at this position
    pf = Profile(typ = 'fon',par = [0.89 - 0.2 * xspan ,0.15 - 0.05 * xspan,0.05,0.05 * (1. - xspan),0.015],npt = 59) # creation of the 2d profile
    pol = pf.polyline(closed = True)  # TODO : Profile should herit of Polyline_2D
    pp = pol.to_frame(f, scale = local_chord)


    index_leading_edge = int(len(pp)/2)-1
    pp.pop_to_geom(geom)
    #shape_extrados, wire_extrados = spline_from_polyline(pp[:index_leading_edge+1])
    #shape_intrados, wire_intrados = spline_from_polyline(pp[index_leading_edge:-1])
    wire = wire_from_polyline(pp[:-1])
    wire_extrados = wire_from_polyline(pp[:index_leading_edge + 1])
    wire_intrados = wire_from_polyline(pp[index_leading_edge:-1])
    wire_trailing_edge = wire_from_polyline([pp[-2], pp[0]])

    # for displaying the vertices
    #for p in pp[:-1]:
    #    pt = BRepBuilderAPI_MakeVertex(gp_Pnt(p[0], p[1], p[2])).Shape()
    #    display.DisplayShape(pt)


    # display the wires
    #display.DisplayShape(wire_extrados.Shape(), update=True)
    #display.DisplayShape(wire_intrados.Shape(), update=True)
    display.DisplayShape(wire.Shape(), update=True)

    generator_extrados.AddWire(wire_extrados.Wire())
    generator_intrados.AddWire(wire_intrados.Wire())
    generator_trailing_edge.AddWire(wire_trailing_edge.Wire())
    generator.AddWire(wire.Wire())

#start_display()
generator_extrados.Build()
extrados_shape = generator_extrados.Shape()
generator_intrados.Build()
intrados_shape = generator_intrados.Shape()
generator_trailing_edge.Build()
trailing_edge_shape = generator_trailing_edge.Shape()
intrados_trailing_edge = generator_trailing_edge.Shape()
display.DisplayShape(extrados_shape)
display.DisplayShape(intrados_shape)
display.DisplayShape(trailing_edge_shape)

generator.Build()
#display.DisplayShape(generator.Shape())
step_writer = STEPControl_Writer()
step_writer.Transfer(extrados_shape, STEPControl_AsIs)
step_writer.Transfer(intrados_shape, STEPControl_AsIs)





ndisc = 80
span = 0.4
rear_pos =  -0.6
max_chord = 0.06
chordf = chord_function(1.)

a_point = [
        Point([0. + rear_pos ,span/float(ndisc),0.]),
        Point([0.01 + rear_pos ,span/4,-0.002]),
        Point([0. + rear_pos ,span/2,-0.01]),
        Point([-0.02 + rear_pos ,2.*span/3.,-0.015]),
        Point([-0.08 + rear_pos ,span,0.02])
            ]


generator = Spline3D(a_point)
fa = generator.frame_array(mode = Vector([0.,0.,1.]), n=ndisc)

#fa = [Frame([[0.,0.,0.], [[0.,0.,1.],[1.,0.,0.],[0.,1.,0.]]])] + fa

shapes = []
generator_extrados = BRepOffsetAPI_ThruSections(False, True)
generator_intrados = BRepOffsetAPI_ThruSections(False, True)
generator_trailing_edge = BRepOffsetAPI_ThruSections(False, True)
generator = BRepOffsetAPI_ThruSections(False, True)
print dir(generator)

chord = lambda x: max_chord * chordf(x)
for i,f in enumerate(fa):
    # adimensioned position along span
    xspan = distance(fa[0][0], f[0])/span

    # dimesioned chord value at xspan
    local_chord = chord(xspan)

    # the profile at this position
    pf = Profile(typ = 'fon',par = [0.89 - 0.2 * xspan ,0.15 - 0.05 * xspan,0.05,-0.05 * (1. - xspan),-0.015],npt = 39) # creation of the 2d profile
    pol = pf.polyline(closed = True)  # TODO : Profile should herit of Polyline_2D
    pp = pol.to_frame(f, scale = local_chord)


    index_leading_edge = int(len(pp)/2)-1
    pp.pop_to_geom(geom)
    #shape_extrados, wire_extrados = spline_from_polyline(pp[:index_leading_edge+1])
    #shape_intrados, wire_intrados = spline_from_polyline(pp[index_leading_edge:-1])
    wire = wire_from_polyline(pp[:-1])
    wire_extrados = wire_from_polyline(pp[:index_leading_edge + 1])
    wire_intrados = wire_from_polyline(pp[index_leading_edge:-1])
    wire_trailing_edge = wire_from_polyline([pp[-2], pp[0]])

    # for displaying the vertices
    #for p in pp[:-1]:
    #    pt = BRepBuilderAPI_MakeVertex(gp_Pnt(p[0], p[1], p[2])).Shape()
    #    display.DisplayShape(pt)


    # display the wires
    #display.DisplayShape(wire_extrados.Shape(), update=True)
    #display.DisplayShape(wire_intrados.Shape(), update=True)
    display.DisplayShape(wire.Shape(), update=True)

    generator_extrados.AddWire(wire_extrados.Wire())
    generator_intrados.AddWire(wire_intrados.Wire())
    generator_trailing_edge.AddWire(wire_trailing_edge.Wire())
    generator.AddWire(wire.Wire())

#start_display()
generator_extrados.Build()
extrados_shape = generator_extrados.Shape()
generator_intrados.Build()
intrados_shape = generator_intrados.Shape()
generator_trailing_edge.Build()
trailing_edge_shape = generator_trailing_edge.Shape()
intrados_trailing_edge = generator_trailing_edge.Shape()
display.DisplayShape(extrados_shape)
display.DisplayShape(intrados_shape)
display.DisplayShape(trailing_edge_shape)

generator.Build()
#display.DisplayShape(generator.Shape())
step_writer.Transfer(extrados_shape, STEPControl_AsIs)
step_writer.Transfer(intrados_shape, STEPControl_AsIs)














#step_writer.Transfer(generator.Shape(), STEPControl_AsIs)
status = step_writer.Write("surface.stp")
start_display()
write_geo('foil',geom)

