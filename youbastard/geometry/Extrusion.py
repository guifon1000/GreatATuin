from youbastard.geometry.Vector import Vector
from youbastard.geometry.Polyline2D import Polyline2D
from youbastard.geometry.Frame import Frame


class Extrusion(dict):
    def __init__(self, polyline, generator, mode = Vector([0.,0.,1.]), n = 50, *largs, **kwargs):
        """
         polyline is wether closed 2D polyline
         or a dict :
        
         {
         'name1' : [opened_polyline 1],
                 ...
         'nameN' : [opened_polyline N]
         }
         the reunion of all these polylines must be closed and have no zero-length segments
        
         generator must be an array of Frames

         along the generator, a parameter is designed by a real s in [0., 1.]

         in kwargs we define FUNCTIONS for ex : 
          scale = lambda s: funcscale(s)
          rotation = lambda s: funcrotation(s)
          translate = lambda s: translate_rotation(s)
          ...
        """
        obj = {}
        frame_array = generator.frame_array(mode = mode, n = n)
        if (type(polyline) == Polyline2D) and polyline.is_closed:
            obj['surface'] = []
            print 'closed polyline'
            for i,frame in enumerate(frame_array):
                s = float(i)/float(n-1)
                if kwargs.has_key('scale'):
                    scale = kwargs['scale'](s) 
                else: scale = 1.
                if kwargs.has_key('rotate'):
                    rotate = kwargs['rotate'](s) 
                else: rotate = 0.
                if kwargs.has_key('translate'):
                    translate = kwargs['translate'](s) 
                else: translate = [0., 0.]
                obj['surface'].append(polyline.to_frame(frame, scale = scale, translate = translate, rotate = rotate)) 
        elif (type(polyline) == dict):
            print 'dico'
            for k in polyline.keys():
                obj[k] = []

            for i,frame in enumerate(frame_array):
                s = float(i)/float(n-1)
                if kwargs.has_key('scale'):
                    scale = kwargs['scale'](s) 
                else: scale = 1.
                if kwargs.has_key('rotate'):
                    rotate = kwargs['rotate'](s) 
                else: rotate = 0.
                if kwargs.has_key('translate'):
                    translate = kwargs['translate'](s) 
                else: translate = [0., 0.]
                for k in polyline.keys():
                    named_polyline = polyline[k]
                    obj[k].append(named_polyline.to_frame(frame, scale = scale, translate = translate, rotate = rotate)) 
        super(Extrusion, self).__init__(obj)

    def pop_to_geom(self, geom):
        for k in self.keys():
            for pol3d in self[k]:
                pol3d.pop_to_geom(geom)
