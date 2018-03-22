from Point import Point
import sys
import numpy as np
from youbastard.geo_functions import cross, is_on_plane
from Vector import Vector

class Plane(list):
    """
    defines a plane 
    """
    def __init__(self, *largs):
        if len(*largs) == 4:
            super(Plane, self).__init__(*largs)
        elif (len(*largs) == 3) and ([type(largs[0][i]) for i in range(3)] == [Point,Point,Point]):
            A = largs[0][0]
            B = largs[0][1]
            C = largs[0][2]
            v0 = Vector([B[i] - A[i] for i in range(3)])
            v1 = Vector([C[i] - A[i] for i in range(3)])
            v = cross(v0,v1)
            super(Plane, self).__init__([v[0], v[1], v[2], -( v[0]*A[0] + v[1]*A[1] + v[2]*A[2] )])
        elif (len(*largs) == 2) and ([type(largs[0][i]) for i in range(2)] == [Point,Vector]):
            A = largs[0][0]
            v = largs[0][1]
            super(Plane, self).__init__([v[0], v[1], v[2], -( v[0]*A[0] + v[1]*A[1] + v[2]*A[2] )])

    @property
    def normal_vector(self):
        return Vector([self[i] for i in range(3)]).unit()


