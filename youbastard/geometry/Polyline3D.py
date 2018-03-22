


class Polyline3D(list):    #always closed
    def __init__(self, *largs,**kwargs):
        super(Polyline3D,self).__init__(*largs)
        self.check_closed = False

    def pop_to_geom(self, geom):
        pts = []
        lns = []
        for p in self:
            print p
            p = geom.add_point(p,0.1)
            pts.append(p)
        for i in range(len(pts)-1):
            l = geom.add_line(pts[i],pts[i+1])
            lns.append(l)
        return lns

