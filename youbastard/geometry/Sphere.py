from Triangulation import Triangulation, read_json_file
from Point import Point
from Vector import Vector
import numpy as np
import matplotlib.pyplot as plt


class Sphere(Triangulation):
    def __init__(self, radius = 1., center = (0.,0.,0.), refin = 1 , **kwargs):
        self.radius = float(radius)
        d = read_json_file('../samples/icosahedron.json')
        d.translate(center)
        for i in range(refin):
            d = d.refine_2()
        _vertices = []
        for p in d['vertices']:
            dist = np.sqrt(np.sum([ (p[i] - d.cg[i])**2. for i in range(3)]))
            _vertices.append(Point([d.cg[i] + (p[i]-d.cg[i])*radius/dist for i in range(3)]))
        self['vertices'] = _vertices
        self['faces'] = d['faces']
        #self['physical'] = [[1, 'default']]
        #self['belongs'] = [1 for i in range(len(self['faces'])) ]
        print self['faces']

    def mercator_map(self):
        self.plane_pos = []
        #plt.clf()
        for j,p in enumerate(self['vertices']):
            adipos = [(p[i] - self.cg[i])/self.radius for i in range(3) ]
            lon = np.arctan2(adipos[1], adipos[0])
            if abs(adipos[2]) < 1. : 
                lat = np.arccos(adipos[2])
            else:
                lat = 0.
            self.plane_pos.append((lon,lat))
            #plt.scatter(lon,lat,c= 'k', s = 1)
        #plt.show()
