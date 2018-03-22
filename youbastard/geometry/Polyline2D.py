import numpy as np
from Point import Point
from Vector import Vector
from Polyline3D import Polyline3D
from youbastard.geo_functions import dot


class Polyline2D(list):    #always closed
    def __init__(self, *largs,**kwargs):
        super(Polyline2D,self).__init__(*largs)
        if kwargs.has_key('closed'):
            if kwargs['closed'] : 
                if not self.is_closed:
                    self.append(self[0])
        if kwargs.has_key('z'):
            self.z=kwargs['z']
        else:
            self.z = 0.
        self.pt3d = []
        if kwargs.has_key('reference_point'):
            self.reference_point = kwargs['reference_point']
        else:
            self.reference_point = [0., 0.]
        for i in range(len(self)):
            p = Point(self[i])
            self.pt3d.append( Point([0. , 0., 0.]))


    @property
    def is_closed(self):
        if self[0] == self[-1]:
            return True
        else:
            return False

    def close(self):
        if self.is_closed:
            print 'the polyline is already closed'
        else:
            self.append(self[0])

    def to_frame(self, frame, **kwargs):
        # frame[1] is X
        # frame[2] is Y
        # frame[3] is Z, and the normal to the plane OXY
        try:
            fac_scale = kwargs['scale']
        except:
            fac_scale = 1.
        try:
            translate =  [kwargs['translate'][i] for i in range(2)]
        except:
            translate = [0., 0.]
        try:
            rotate =  kwargs['rotate']
        except:
            rotate = 0.


        pol3d = []
        origin = frame[0]
        basis = frame[1]
        rot_p = []
        if rotate != 0.:
            for p in self:
                v = Vector([p[0], p[1], 0.])
                rpx = dot(v, Vector([np.cos(rotate), np.sin(rotate), 0.]))
                rpy = dot(v, Vector([-np.sin(rotate), np.cos(rotate), 0.]))
                rot_p.append([rpx, rpy])
        else:
            rot_p=[[p[0], p[1]] for p in self]
        for p in rot_p:
            x3d = origin[0] + fac_scale * ( (p[0] - translate[0]) * dot(basis[1],(1.,0.,0.)) +
                                             (p[1] - translate[1]) * dot(basis[2],(1.,0.,0.)))
            y3d = origin[1] + fac_scale * ( (p[0] - translate[0]) * dot(basis[1],(0.,1.,0.)) +
                                             (p[1] - translate[1]) * dot(basis[2],(0.,1.,0.)))
            z3d = origin[2] + fac_scale * ( (p[0] - translate[1]) * dot(basis[1],(0.,0.,1.)) +
                                             (p[1] - translate[1]) * dot(basis[2],(0.,0.,1.)))
            pol3d.append(Point([x3d, y3d, z3d]))
        return Polyline3D(pol3d)


    def pop_to_geom(self, geom):
        pts = []
        lns = []
        if self.is_closed:
            for i,p in enumerate(self[:-1]):
                p = geom.add_point(p,0.1)
                pts.append(p)
            for i in range(len(pts)-1):
                l = geom.add_line(pts[i],pts[i+1])
                lns.append(l)
           
            l = geom.add_line(pts[-1], pts[0])
            lns.append(l)
        return lns

    def plot(self):
        import matplotlib.pyplot as plt
	plt.plot([p[0] for p in self], [p[1] for p in self])
	plt.axis('equal')
	plt.show()


class Circle(Polyline2D):
    def __init__(self, center=[0., 0.], radius=1., nseg=20, closed=True):
        pts = []
	for i in range(nseg):
	    teta = 2.*np.pi * float(i) /float(nseg)
	    pts.append(
	        [center[0] + radius * np.cos(teta),
		center[1] + radius * np.sin(teta)]
		)
	super(Circle,self).__init__(pts, closed = closed) 



        
