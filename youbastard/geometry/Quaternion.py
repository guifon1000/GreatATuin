import math
import numpy as np
from Vector import Vector
from youbastard.geo_functions import dot, cross

class Quaternion(list):

    """
    Quaternion = (w 1, x i, y j, z k)
                  ^    ^    ^    ^
                  s    v    v    v   
                  
                  (v = vectorial part, s = scalar part)
    """
    def __init__(self, *largs):
        if len(*largs) == 4:
            super(Quaternion, self).__init__(*largs)
        else:
            super(Quaternion, self).__init__([0., 0., 0., 0.])

    @property
    def scalar(self):
        return self[0]


    @property
    def vector(self):
        return Vector( (self[1], self[2], self[3]))

    @property
    def conjugate(self):
        """
        conj(q) = ( q[0] , -q[1] , -q[2] , -q[3] )
        """
        return Quaternion( (self[0], -self[1], -self[2], -self[3]) )


    def __str__(self):
        aff='('
        aff+=str(self[0])+')+('
        aff+=str(self[1])+')i+('
        aff+=str(self[2])+')j+('
        aff+=str(self[3])+')k'
        return aff

    def __neg__(self):
        return Quaternion([-self[i] for i in range(4)])

    def __add__(self,other):
        return Quaternion( [self[i] + other[i] for i in range(4)] )

    def __sub__(self,other):
        return Quaternion( [self[i] - other[i] for i in range(4)] )



    def __abs__(self):
        n2 = self * self.conjugate
        return np.sqrt(n2[0]) 


    def __mul__(self,other):
        """
        uses (sa,va) * (sb,vb) = (sa*sb - dot(va,vb)     , cross(va,vb) + sa*vb + sb*va )
                                  SCALAR PART         VECTOR PART 
        where s is scalar part, v is vectorial part (a Vector)
        """
        scalar = self.scalar * other.scalar - dot(self.vector, other.vector)
        vector = cross(self.vector, other.vector) + self.scalar*other.vector + other.scalar*self.vector
        return Quaternion( (scalar,vector[0],vector[1],vector[2]) )

    def __rmul__(self, rval):
        """
        multiplication by a real scalar rval
        """
        return Quaternion( [rval*self[i] for i in range(4)])

    def unit(self):
        if abs(self) != 0. :
            return Quaternion( [self[i]/abs(self) for i in range(4) ] )
        else:
            return 0.

    def inverse(self):
        if abs(self) != 0. :
            conj = self.conjugate
            norm2 = abs(self)**2.
            return Quaternion( [conj[i]/norm2 for i in range(4) ] )
        else:
            return 0.

################################################################################
    def __div__(self,other):
        return self*(1./abs(other)**2*other.conjugate())

    def __pow__(self,n):
        r=1
        for i in range(n):
            r=r*self
        return r
    def to_matrix(self):
        mat=np.zeros((3,3),dtype=np.float)
        unit = self.unit()
        w = unit[0]
        x = unit[1]
        y = unit[2]
        z = unit[3]
        mat[0,0]=1.-2*y**2.-2.*z**2. 
        mat[0,1]=2.*x*y-2.*z*w
        mat[0,2]=2.*x*z+2.*y*w
        mat[1,0]=2.*x*y+2.*z*w
        mat[1,1]=1.-2*x**2.-2.*z**2.
        mat[1,2]=2.*y*z-2.*x*w
        mat[2,0]=2.*x*z-2.*y*w
        mat[2,1]=2.*y*z+2.*x*w
        mat[2,2]=1.-2*x**2.-2.*y**2.
        return mat

def matrix_to_quaternion(mat):
    # mat is a 3x3 matrix
    m00 = m[0][0]
    m01 = m[0][1]
    m02 = m[0][2]
    m10 = m[1][0]
    m11 = m[1][1]
    m12 = m[1][2]
    m20 = m[2][0]
    m21 = m[2][1]
    m22 = m[2][2]
    
    w = math.sqrt( max( 0., 1. + m00 + m11 + m22 ) ) / 2. 
    x = math.sqrt( max( 0., 1. + m00 - m11 - m22 ) ) / 2. 
    y = math.sqrt( max( 0., 1. - m00 + m11 - m22 ) ) / 2. 
    z = math.sqrt( max( 0., 1. - m00 - m11 + m22 ) ) / 2. 
    x = math.copysign( x, m21 - m12 ) 
    y = math.copysign( y, m02 - m20 ) 
    z = math.copysign( z, m10 - m01 ) 
    return Quaternion((w, x, y, z)) 

if __name__=='__main__':
    q=Quaternion((0.5,-0.5,-0.5,0.5) )
    print q
    print '-------------------------------'
    m = q.to_matrix()
    print m
    print '-------------------------------'
    print matrix_to_quaternion(m)
