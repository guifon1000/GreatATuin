
from scipy import interpolate
from Vector import Vector
from Point import Point
import sys 
sys.path.append('../../')
import numpy as np


class Frame(list):
    """
    if (T,N,B) is a frame placed in M, the given list is :
    [ 
      [Mx, My, Mz],
      [Tx, Ty, Tz],
      [Nx, Ny, Nz],
      [Bx, By, Bz] ]
    """
    def __init__(self, *largs):
	super(Frame, self).__init__(*largs)




