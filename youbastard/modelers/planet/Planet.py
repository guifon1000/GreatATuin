from youbastard.geometry.Sphere import Sphere
import random
import numpy as np

class Planet(Sphere):
    def __init__(self, center = (0., 0., 0.), radius = 1., refin =3):
        super(Planet, self).__init__(radius, center, refin)
        self.mercator_map()
        f = [ 1. for i in range(len(self['vertices'])) ]
        things = []
        _vertices = []
        hgt = 0.3*self.radius
        lat_geo = 0.000001*np.pi/2
        lon_geo = 0.
        sigma = 0.5
        for i,p in enumerate(self['vertices']):
            lat = self.plane_pos[i][1]
            lon = self.plane_pos[i][0]
            f[i] += hgt * np.exp(-(1./sigma**2.) * ((lat-lat_geo)**2. + (lon-lon_geo)**2.  ))
            #f[i] += hgt * np.exp(-(1./sigma**2.) * ((lat-lat_geo-2.*np.pi)**2. + (lon-lon_geo)**2.  ))
            #f[i] += hgt * np.exp(-(1./sigma**2.) * ((lat-lat_geo+2.*np.pi)**2. + (lon-lon_geo)**2.  ))
            #north_lon = 2.* np.pi - lon_geo
            #south_lon = - lon_geo
            #f[i] += hgt * np.exp(-(1./sigma**2.) * ((lat-lat_geo)**2. + (lon-north_lon)**2.  ))
            #f[i] += hgt * np.exp(-(1./sigma**2.) * ((lat-lat_geo)**2. + (lon-south_lon)**2.  ))
            p0 = [(p[j] - self.cg[j]) for j in range(3)]
            _vertices.append([self.cg[j] + p0[j]*f[i] for j in range(3) ] )
        self['vertices'] = _vertices

