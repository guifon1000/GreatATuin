import sys
import numpy as np
from Vector import Vector
from Point import Point
from Plane import Plane
sys.path.append('../')
from youbastard.geo_functions import cross, intersect_3_planes

lcar = 0.1

class Triangle(list):
    """    
               
                self[2]
                   ^
                 /  \
                /    \
           s3  /      \ s0
              /        \
             /     0    \
            v            \
     self[0] -----------> self[1]    (  0 = out normal )
                   s1
    """
    def __init__(self, *largs):
        super(Triangle, self).__init__(*largs)

    
    def __neg__(self):
        return Triangle((self[0], self[2], self[1]))



    @property
    def cg(self):
        return Point(np.mean(self   ,axis=0))

    @property
    def normal(self):
        u = Vector((self[1][0]-self[0][0],\
    			self[1][1]-self[0][1],\
    			self[1][2]-self[0][2]))
        v = Vector((self[2][0]-self[1][0],\
    			self[2][1]-self[1][1],\
     			self[2][2]-self[1][2]))
        return cross(u,v)

    @property
    def circumcenter(self):
        plane_1 = Plane(self)
        mid_1 = Point([0.5 * (self[0][i] + self[1][i]) for i in range(3)])
        vnor_1 = Vector([mid_1[i] - self[0][i] for i in range(3)]).unit()
        plane_2 = Plane([mid_1, vnor_1])
        mid_2 = Point([0.5 * (self[0][i] + self[2][i]) for i in range(3)])
        vnor_2 = Vector([mid_2[i] - self[0][i] for i in range(3)]).unit()
        plane_3 = Plane([mid_2, vnor_2])
        print '-------------'
        print plane_1
        print '-------------'
        print plane_2
        print '-------------'
        print plane_3
        return intersect_3_planes(plane_1, plane_2, plane_3)
