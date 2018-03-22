import numpy as np
from Point import Point
from Vector import Vector
from Line import Line
from Polyline2D import Polyline2D
from youbastard.geo_functions import angle, cross, intersect_2_lines

class TreeNode(list): 
    def __init__(self, *largs,**kwargs):
        if len(largs[0])>1:
            super(TreeNode,self).__init__(*largs)
            self.center = self[0]
            self.ref_vect = Vector([self[1][i] - self.center[i] for i in range(3)]).unit()

    def set_thicknesses(self, *largs):
        if len(largs[0]) == len(self)-1:
            self.thick = [float(t) for t in largs[0]]
        else:
            print 'the length of the thickness vector does not fit'
            sys.exit()

    def reorder_sectors(self):
        def sec_angle(p):
            Zvec = Vector([0., 0., 1.])
            v = Vector([p[i] - self.center[i] for i in range(3)]).unit()
            ang = angle(self.ref_vect, v, plane_normal=Zvec)
            if ang<0.:
                ang = 2*np.pi + ang
            return ang
        if len(self)>2:
            if not hasattr(self,'thick'):
                order = [self[1]]+sorted(self[2:], key = sec_angle)+[self[1]]
                super(TreeNode,self).__init__(order)
                #print order
            else:
                temp = sorted(zip(self[1:], self.thick), key=lambda x: sec_angle(x[0]))
                order,self.thick = map(list, zip(*temp))
                order = [self.center] + order
                #self.order = order
                #self = order
                super(TreeNode,self).__init__(order)

    def offset(self, default_thickness = None):
        points = []
        self.reorder_sectors()
        if default_thickness:
            thck = [float(default_thickness) for i in range(len(self))]
        elif hasattr(self, 'thick'):
            thck = self.thick + [self.thick[0]]
        if len(self)>2:
            order = self[1:]+[self[1]]
            print '--ordered points--'
            print order
            print '--sectors--'
            sectors = [(order[i],self.center,order[i+1],thck[i],thck[i+1]) for i in range(len(order)-1)] 
            print sectors
            print '----'

            #prev = None
            for isec,s in enumerate(sectors):
                #print s
                v1 = Vector([s[0][i]-s[1][i] for i in range(3)]).unit()
                v2 = Vector([s[2][i]-s[1][i] for i in range(3)]).unit()
                t1 = cross(Vector([0., 0., 1.]), v1).unit()
                t2 = cross(v2, Vector([0., 0., 1.])).unit()
                p1 = Point([s[0][i] + 0.5 * s[3] * t1[i] for i in range(3)])
                p2 = Point([s[2][i] + 0.5 * s[4] * t2[i] for i in range(3)])
                li1 = Line([p1,v1])
                li2 = Line([p2,v2])
                p0 = intersect_2_lines(li1,li2)
                #gp1 = p1.pop_to_geom(geom)
                #gp2 = p2.pop_to_geom(geom)
                #if prev:
                #    gprev = prev.pop_to_geom(geom)
                #    geom.add_line(gprev,gp1)
                if p0:
                    #gp0 = p0.pop_to_geom(geom)
                    #geom.add_line(gp1,gp0) 
                    #geom.add_line(gp0,gp2)
                    #plt.plot([p1[0], p0[0], p2[0]],
                    #         [p1[1], p0[1], p2[1]])
                    points.append(p1)
                    points.append(p0)
                    points.append(p2)
                else:
                    points.append(p1)
                    points.append(p2)

                    #geom.add_line(gp1,gp2)
        else:
            th = thck[0]
            v1 = Vector([self[1][i]-self[0][i] for i in range(3)]).unit()
            t1 = cross(Vector([0., 0., 1.]), v1).unit()
            p1 = Point([self[1][i] + 0.5 * th * t1[i] for i in range(3)])
            p2 = Point([self[0][i] + 0.5 * th * t1[i] for i in range(3)])
            p3 = Point([self[0][i] - 0.5 * th * t1[i] for i in range(3)])
            p4 = Point([self[1][i] - 0.5 * th * t1[i] for i in range(3)])
            points = [p1, p2, p3, p4]
        return Polyline2D(points,closed = True)

