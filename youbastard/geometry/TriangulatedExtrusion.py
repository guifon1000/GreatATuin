from Frame import Frame
from Triangulation import Triangulation, read_msh_file
from youbastard.io_functions import write_geo
import pygmsh as pg

class TriangulatedExtrusion(Triangulation):
    """

                       |                    |
     l_first[i].points[1]                     l_second[i].points[1]
                       +--------------------+
                       |         l3         |
                    ^  |      <-----        |  ^
        l_first[i]  |  |                    |  |   l_second[i]
        (segment)   |  |  |             ^   |  |   (segment)
                    |  |  |l0        l2 |   |  |
                    +  |  V             |   |  +
                       |      ----->        |
    l_first[i].points[0]         l1         |
                       +--------------------+  l_second[i].points[0]

                       |     --->----->     |
                              extrusion
    """

    def __init__(self, pol2D, frames, name = 'TMP', **kwargs):
        pol = pol2D
        # If it is closed, the first and last points are the same
        close_polyline = False
        if pol.is_closed :
            pol.pop()
            close_polyline = True
            
        # the number of points of the polyline. 
        Npt = len(pol)

        dphys = {}

        if ('mode' in kwargs) and (kwargs['mode'] == 'profile'):
            dphys['extrados'] = {'indices': [0, Npt/2], 'surfaces': []}
            dphys['intrados'] = {'indices': [Npt/2, Npt], 'surfaces': []}
        dphys['default'] = {'surfaces': []}
         
        geom = pg.built_in.Geometry()
        # there is one more quaternion/frame than slices
        Nslices = len(frames) - 1

        if 'close_caps' in kwargs:
            if kwargs['close_caps'] == True:
                cap_ends = True
            else:
                cap_ends = False

        if ('scales' in kwargs) and (len(kwargs['scales']) == len(frames)):
            scales = kwargs['scales']
        else:
            scales = [1. for i in range(len(frames))]

        # get the first polyline
        first = pol.to_frame(frames[0], scale=scales[0])

        l_first = first.pop_to_geom(geom) 
        l_second = None
        # l_first contains the GMSH segments of the first polyline
        # the j-th segment's first/second point l_first[j].points[0/1] ...

        # FIRST CAP (IF POLYLINE IS CLOSED)
        if cap_ends and close_polyline:
            loop = []
            for iseg, segment in enumerate(l_first) :
                loop.append(segment)
            loop.append(geom.add_line(
                l_first[-1].points[1] , 
                l_first[0].points[0])
                )
            lloop_first = geom.add_line_loop(loop)
            cap_first = geom.add_plane_surface(lloop_first)

            dphys['default']['surfaces'].append(cap_first)

        for i in range(Nslices):
            second = pol.to_frame(frames[i+1], scale = scales[i+1])
            l_second = second.pop_to_geom(geom)

            # extrusion of the OPEN polyline 
            for j in range(len(l_first) ) :
                l0 = -l_first[j]
                l1 = geom.add_line(l_first[j].points[0], l_second[j].points[0])
                l2 = l_second[j]
                l3 = geom.add_line(l_second[j].points[1], l_first[j].points[1])

                lloop = geom.add_line_loop([l0, l1, l2, l3])
                sf = geom.add_surface(lloop)
                
                to_physical = False
                for k in dphys.keys():
                    if ('indices' in dphys[k]) and (len(dphys[k]['indices'])==2) :
                        start = min(dphys[k]['indices'])
                        end = max(dphys[k]['indices'])
                        if (j>=start) and (j<end):# append the surface to the right physical group...
                            dphys[k]['surfaces'].append(sf)
                            to_physical = True

                if not to_physical:
                    dphys['default']['surfaces'].append(sf)

            if close_polyline:
                # CLOSE THE POLYLINE
                l0 = geom.add_line( l_first[0].points[0] , 
                                    l_first[-1].points[1] ) 
                l1 = geom.add_line( l_first[-1].points[1] , 
                                    l_second[-1].points[1] ) 
                l2 = geom.add_line( l_second[-1].points[1] , 
                                    l_second[0].points[0] ) 
                l3 = geom.add_line( l_second[0].points[0] , 
                                    l_first[0].points[0] ) 
                lloop = geom.add_line_loop([l0, l1, l2, l3])
                sf = geom.add_surface(lloop)
                dphys['default']['surfaces'].append(sf)

            first = second
            l_first = l_second
        
        if cap_ends and close_polyline:
        # LAST CAP (IF POLYLINE IS CLOSED)
        # l_second is now the last polyline : 
            loop = []
            for segment in l_second :
                loop.append(-segment)
            loop.append(geom.add_line(
                l_second[0].points[0], 
                l_second[-1].points[1] ))
            lloop_last = geom.add_line_loop(loop)
            cap_last = geom.add_plane_surface(lloop_last)
            dphys['default']['surfaces'].append(cap_last)

        for k in dphys.keys():
            geom.add_physical_surface(dphys[k]['surfaces'], label=k) 

        write_geo(name, geom)
        import subprocess
        exe_gmsh = '/home/fon/gmsh-3.0.3-git-Linux/bin/gmsh'
        subprocess.call(
                [exe_gmsh, name+'.geo', '-2', '-o', 
                name+'.msh', '>', name+'.mshlog'])
        tri = read_msh_file(name)
        super(TriangulatedExtrusion, self).__init__(
                vertices=tri['vertices'],
                faces=tri['faces'])
