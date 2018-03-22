class Wing3D(object):
    def __init__(**kwargs):
        # profiles (may be a function giving a 2D profile as a function of s in [0., 1.])
        # generator = array of Frames
        # twist = n array of angles (2D rotations of the profiles around the tangent of generator)
        # span
        # max_chord
        # chord (constant, n_values or function on [0., 1.]
        # twist (constant, n_values or function on [0., 1.]
        generator_extrados = BRepOffsetAPI_ThruSections(False, True)
        generator_intrados = BRepOffsetAPI_ThruSections(False, True)
        generator_trailing_edge = BRepOffsetAPI_ThruSections(False, True)

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


