from Point import Point
from Vector import Vector
from Line import Line
from youbastard.geo_functions import cross, intersect_2_lines, intersect_2_segments, distance
class Segment(list):
    """
    defines a segment 
    """
    def __init__(self, *largs):
        if ([type(largs[0][i]) for i in range(len(*largs))] == [Point, Point]):
            super(Segment, self).__init__(*largs)
    def line(self):
        return Line([self[0], Vector([self[1][i]-self[0][i] for i in range(3)]).unit()])

class OffsetSegment(object):
    def __init__(self, seg_0, thickness):
        self.generator_segment = seg_0
        mid0 = Point([ 0.5 * (seg_0[0][i] + seg_0[1][i]) for i in range(3)])
        self.thickness = thickness
        vec = Vector([seg_0[1][i] - seg_0[0][i] for i in range(3)]).unit()
        self.trans = cross(vec, Vector((0., 0., 1.))).unit()
        pmid1 = Point([mid0[i] + 0.5 * self.thickness * self.trans[i] for i in range(3)])
        pmid2 = Point([mid0[i] - 0.5 * self.thickness * self.trans[i] for i in range(3)])
        self.l1 = Line((pmid1, vec))
        self.l2 = Line((pmid2, vec))
        self.candidates_1 = []
        self.candidates_2 = []

    def pop_to_geom(self, geom):
        #for p in self.generator_segment:
            #p.pop_to_geom(geom)
        extreme_parameter = 10.
        pt11 = self.l1.parameter_point(-extreme_parameter).pop_to_geom(geom)
        pt12 = self.l1.parameter_point(extreme_parameter).pop_to_geom(geom)
        geom.add_line(pt11, pt12)
        pt21 = self.l2.parameter_point(-extreme_parameter).pop_to_geom(geom)
        pt22 = self.l2.parameter_point(extreme_parameter).pop_to_geom(geom)
        geom.add_line(pt21, pt22)

    def update_candidates(self, others):
        for i,otheri in enumerate(others):
            # first wall of self, first wall of other
            pt11 = intersect_2_lines(self.l1, otheri.l1)
            seg11 = Segment([self.l1[0], pt11])
            app11 = False
            # first wall of self, second wall of other
            pt12 = intersect_2_lines(self.l1, otheri.l2)
            seg12 = Segment([self.l1[0], pt12])
            app12 = False
            # second wall of self, first wall of other
            pt21 = intersect_2_lines(self.l2, otheri.l1)
            seg21 = Segment([self.l2[0], pt21])
            app21 = False
            # second wall of self, second wall of other
            pt22 = intersect_2_lines(self.l2, otheri.l2)
            seg22 = Segment([self.l2[0], pt22])
            app22 = False
            for j,otherj in enumerate(others):
                if i!=j:
                    if intersect_2_lines(seg11.line(), otherj.l1):
                        app11 = True
                    if intersect_2_lines(seg11.line(), otherj.l2):
                        app11 = True
                    if intersect_2_lines(seg12.line(), otherj.l1):
                        app12 = True
                    if intersect_2_lines(seg12.line(), otherj.l2):
                        app12 = True
                    if intersect_2_lines(seg21.line(), otherj.l1):
                        app21 = True
                    if intersect_2_lines(seg21.line(), otherj.l2):
                        app21 = True
                    if intersect_2_lines(seg22.line(), otherj.l1):
                        app22 = True
                    if intersect_2_lines(seg22.line(), otherj.l2):
                        app22 = True


            if app11:self.candidates_1.append(pt11)
            if app12:self.candidates_1.append(pt12)
            if app21:self.candidates_2.append(pt21)
            if app22:self.candidates_1.append(pt22)
        def dst1(pt):
            return distance(pt, self.l1[0])
        def dst2(pt):
            return distance(pt, self.l2[0])
        self.candidates_1 = [sorted(self.candidates_1, key = dst1)[0]]
        self.candidates_2 = [sorted(self.candidates_2, key = dst2)[0]]
        print self.candidates_1


