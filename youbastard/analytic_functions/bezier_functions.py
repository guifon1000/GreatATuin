import numpy as np
from scipy.special import binom as spbinom

def binom(n,k):
    if k<=n:
        return factorial(n)/(factorial(k)*factorial(n-k))

def pascal_triangle(level):
    for i in range(level):
        s=''
        for j in range(i+1):
            s+=str(binom(i,j))+' '
        print s

def factorial(n):
    if n==1 or n==0:
        return 1
    else:
        return n*factorial(n-1)


def de_casteljau(control_points, parameter):
    def elementary_step(array):
        out_array=[]
        for i,p in enumerate(array[:-1]):
            pt = [p[j] * (1.-parameter) + array[i+1][j] * parameter for j in range(2)]
            out_array.append(pt)
        return out_array
    ar = control_points
    for i in range(len(control_points)-1):
        ar = elementary_step(ar)
    return  ar[0]


def piecewise_bezier_polyline(start_value = 0., end_value = 0., *largs):
    import matplotlib.pyplot as plt
    from youbastard.geometry.Point import Point
    from youbastard.geometry.Vector import Vector
    from youbastard.geometry.Line import Line
    from youbastard.geo_functions import cross, dot, angle, intersect_2_lines
    prev = [0., start_value]
    last = [1., end_value]
    bezier_parts = [[prev]]
    plt.clf()
    plt.axis('equal')
    plt.scatter([prev[0], last[0]],[prev[1], last[1]], c= 'k')
    for i,cpl in enumerate(largs[0]):
        if i == len(largs[0])-1:
            nex = last
        else:
            nex = largs[0][i+1]
        pt = [cpl[0], cpl[1]]
        tmppt = Point([pt[0], pt[1], 0.])
        plt.scatter(pt[0], pt[1], c='k')
        rad = cpl[2]
        vec1 = Vector([pt[0] - prev[0], pt[1] - prev[1], 0.])
        vec2 = Vector([nex[0] - pt[0], nex[1] - pt[1], 0.])
        Z = Vector([0., 0., 1.])
        ang = angle(vec1, vec2, plane_normal=Z)
        if ang>0.:sgn = 1.
        elif ang<0.:sgn = -1.
        print ang
        norm1 = sgn * cross(Z,vec1).unit()
        norm2 = sgn * cross(Z,vec2).unit()
        l1 = Line([Point([prev[0], prev[1] , 0.]), vec1])
        l2 = Line([Point([pt[0], pt[1], 0.]), vec2])
        ln1 = Line([Point([prev[0] + norm1[0] * rad, prev[1] + norm1[1] * rad, 0.]),
                    vec1])
                          
        
        ln2 = Line([Point([nex[0] + norm2[0] * rad, nex[1] + norm2[1] * rad, 0.]),
                    vec2])
        
        center_arc = intersect_2_lines(ln1, ln2)
        if center_arc:
            cen = [center_arc[j] for j in range(2)]
            lnorm1 = Line([center_arc, -norm1])
            lnorm2 = Line([center_arc, -norm2])
            start = intersect_2_lines(lnorm1,l1)

            end = intersect_2_lines(lnorm2,l2)

            vecstart = Vector([start[i]-tmppt[i] for i in range(3)])
            vecend = Vector([end[i]-tmppt[i] for i in range(3)])
            if vecstart.norm > 0.5 * vec1.norm:
                print 'too long'
                tmpvec = vecstart.unit()
                start = Point([tmppt[i] + 0.5 * vec1.norm * tmpvec[i] for i in range(3)])
            if vecend.norm > 0.5 * vec2.norm:
                tmpvec = vecend.unit()
                end = Point([tmppt[i] + 0.5 * vec2.norm * tmpvec[i] for i in range(3)])
            bezier_parts[-1].append([start[0], start[1]])
            bezier_parts.append([[start[0], start[1]],[pt[0], pt[1]], [end[0], end[1]]])
            bezier_parts.append([[end[0], end[1]]])
            plt.scatter([start[0], end[0]], [start[1], end[1]], c='g')
        else:
            cen = pt
            bezier_parts[-1].append([pt[0], pt[1]])
            bezier_parts.append([[pt[0], pt[1]]])

        plt.scatter(cen[0], cen[1], c = 'r')

        prev = pt
    bezier_parts[-1].append([last[0], last[1]])

    polx = []
    poly = []

    for bp in bezier_parts:
        polx+=[p[0] for p in bp]
        poly+=[p[1] for p in bp]
    plt.plot(polx, poly)
    def func(parameter):
        ctpts = None
        for i,bp in enumerate(bezier_parts):
            #print parameter,bp[0][0],bp[-1][0], len(bp)
            if (bp[0][0]<=parameter) and (parameter<=bp[-1][0]):
                ctpts = bp
                break
        s = (parameter-ctpts[0][0])/(ctpts[-1][0] - ctpts[0][0])
        return de_casteljau(ctpts, s)[1]
    N = 1000
    x = [float(i)/float(N-1) for i in range(N)] 
    fx = [func(s) for s in x]
    plt.plot(x,fx, '.')
    plt.show()
    return func
    

def de_boor(control_points, knots, parameter):
    if len(knots) >= len(control_points):
        print 'knots : '+str(knots)
        print 'control : '+str(control_points)

    
def chord2_function(p1, p2, p3):
    def fun(parameter):
        if parameter <= p1:
            return (p2-1.)*parameter/p1


def minimized_bezier_function(control_points, value):
    def fun(parameter):
        return min(de_casteljau(control_points,parameter)[1], value)
    return fun


def bezier_function(control_points):
    def fun(parameter):
        return de_casteljau(control_points,parameter)[1]
    return fun

def chord_function(p1):
    pts = [[0., 1.], [p1, 1.], [0.5 * (p1 +1.), (.5 + p1) ], [1.1, 0.5]]
    import matplotlib.pyplot as plt
    plt.plot([p[0] for p in pts], [p[1] for p in pts])
    plt.axis('equal')
    plt.show()
    return minimized_bezier_function(pts, 1.)


def Bernstein(n,k):
    coeff=spbinom(n,k)
    def _bpoly(x):
        return coeff*x**k*(1-x)**(n-k)
    return _bpoly


def Bezier(points, num=100):
    N=len(points)
    t=np.linspace(0,1,num=num)
    curve=np.zeros((num,2))
    for ii in range(N):
        curve+=np.outer(Bernstein(N-1,ii)(t),points[ii])
    return curve


if __name__=='__main__':

    n = 40
    print str(n)+'!'
    print factorial(n)
    n = 7
    print '--------'
    print 'pascal triangle level '+str(n)
    pascal_triangle(n)
    points = [[0.,0.], [0.,1.]]
    print '--------'
    print 'bezier curve '+str(points)
    import matplotlib.pyplot as plt
    plt.scatter([p[0] for p in points],[p[1] for p in points])
    plt.plot([p[0] for p in points],[p[1] for p in points])
    plt.axis('equal')
    sol = []
    N = 1000
    for i in range(N):
        parameter = float(i)/float(N-1)
        sol.append(de_casteljau(points, parameter))
    plt.plot([p[0] for p in sol],[p[1] for p in sol])
    plt.show()
    #points = chord_function(0.5, 0.9, 1.) 
    #fun = bezier_function(points)
    fun = chord_function(.8)
    sol = []
    l_over_d = 5.
    for i in range(N):
        parameter = float(i)/float(N-1)
        sol.append([parameter*l_over_d,fun(parameter)])
    plt.clf()
    plt.axis('equal')
    #plt.scatter([p[0] for p in points],[p[1] for p in points])
    #plt.plot([p[0] for p in points],[p[1] for p in points])
    plt.plot([p[0] for p in sol],[p[1] for p in sol])
    plt.show()

