import numpy as np
import pygmsh as pg


class Vector(list):
    def __init__(self, *largs):
        super(Vector,self).__init__(*largs)

    def __rmul__(self,rval):
        return Vector( [rval*self[i] for i in range(len(self)) ] )

    def __add__(self,other):
        return Vector( [self[i] + other[i] for i in range(len(self)) ] )
    
    def __neg__(self):
	return Vector( [-self[i] for i in range(len(self))] )

    @property
    def norm(self):
        return np.sqrt(self.norm2)

    @property
    def norm2(self):
        return np.sum([c**2. for c in self])

    def unit(self):
        return Vector( [ self[i]/self.norm for i in range(3)])

