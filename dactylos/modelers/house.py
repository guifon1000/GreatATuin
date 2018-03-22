from youbastard.geometry.Polyline2D import Polyline2D
from youbastard.geometry.Point import Point
from youbastard.geometry.Vector import Vector
from youbastard.geometry.Frame import Frame
from youbastard.geometry.Plane import Plane
from youbastard.geometry.Line import Line
from youbastard.geometry.Segment import Segment
from youbastard.geo_functions import cross
from dactylos.cad_functions import extrude_polyline2d, show_shapes

def rectangle(width, height):
    pt1 = [- 0.5 * height,   0.5 * width]
    pt2 = [- 0.5 * height, - 0.5 * width]
    pt3 = [  0.5 * height, - 0.5 * width]
    pt4 = [  0.5 * height,   0.5 * width]
    return Polyline2D([pt1, pt2, pt3, pt4], closed = True)

class Wall(Segment):
    def __init__(self, pt1, pt2, thickness, vertical_vector):
        super(Wall, self).__init__([pt1, pt2])
        vert = vertical_vector.unit()
        wall_direction = Vector([pt2[i] - pt1[i] for i in range(3)]).unit()
        normal_direction = cross(wall_direction, vertical_vector).unit() 
        self.matrix = [
                      [normal_direction[i] for i in range(3)],
                      [vert[i] for i in range(3)],
                      [wall_direction[i] for i in range(3)]
                      ]
        self.thickness = thickness
        frame = Frame( [pt1 , self.matrix]) 

class Opening(object):
    def __init__(self, pt1, pt2, thickness, dist_from_pt1, height, vertical_vector, polyline2d, close = False):
        self.close = None
        epsilon = 0.01
        wall = Wall(pt1, pt2, thickness, vertical_vector)
        bottom_center = Point([pt1[i] + dist_from_pt1 * wall.matrix[2][i] for i in range(3)])
        center = Point([
            bottom_center[i] 
            + height * wall.matrix[1][i] 
            - 0.5 * (thickness + epsilon) * wall.matrix[0][i] 
            for i in range(3)])
        frame = Frame( [center , wall.matrix]) 
        self.ex = extrude_polyline2d(polyline2d, frame, thickness+epsilon)
	if close:
            center = Point([
                bottom_center[i] 
                + height * wall.matrix[1][i] 
                - 0.5 * (0.25 * thickness) * wall.matrix[0][i] 
                for i in range(3)])
            frame = Frame( [center , wall.matrix]) 
            self.close = extrude_polyline2d(polyline2d, frame, 0.5 * thickness)
        

class BoxOnWall(object):
    def __init__(self, pt1, pt2, wall_thickness, box_thickness, box_offset, dist_from_pt1, height, vertical_vector, polyline2d):
        epsilon = 0.01
        wall = Wall(pt1, pt2, wall_thickness, vertical_vector)
        bottom_center = Point([pt1[i] + dist_from_pt1 * wall.matrix[2][i] for i in range(3)])
        center = Point([
            bottom_center[i] 
            + height * wall.matrix[1][i] 
            +(0.5 * wall_thickness + box_offset) * wall.matrix[0][i] 
            for i in range(3)])
        frame = Frame( [center , wall.matrix]) 
        self.ex = extrude_polyline2d(polyline2d, frame, box_thickness)
 


