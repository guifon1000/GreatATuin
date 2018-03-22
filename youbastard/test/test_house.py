from youbastard.geometry.Point import Point 
from youbastard.geometry.Vector import Vector 
from youbastard.geometry.TreeNode import TreeNode 
from youbastard.geo_functions import distance
from youbastard.io_functions import write_polylines_sets


data = {'points': [Point([-5., 2.5, 0.]), 
                    Point([5., 2.5, 0.]), 
                    Point([5., -2.5, 0.]),
                    Point([-5., -2.5, 0.])],
        'walls': [[0,1], [1,2], [2,3], [3,0]]}

data = {'points': [Point([0., 0., 0.]), 
                    Point([4., 0., 0.]), 
                    Point([8., 0., 0.]), 
                    Point([11., 0., 0.]), 
                    Point([0., 2., 0.]), 
                    Point([4., 2., 0.]), 
                    Point([6., 2., 0.]), 
                    Point([8., 2., 0.]), 
                    Point([11., 2., 0.]), 
                    Point([0., 5., 0.]),
                    Point([4., 3., 0.]),
                    Point([6., 3., 0.]),
                    Point([6., 5., 0.]),
                    Point([11., 5., 0.])],
        'walls': [
            [0,1], [1,2], [2,3], [3,8], [8,7], [7,2], [7,6], [6,11], [11,10], [10,5],
            [5,1], [5,4], [0,4], [4,9], [9,12], [12,11], [12,13], [13,8], [1,6], [6,13]
            ]
        }

polylines = []
all_tree_nodes = []
thick = 0.2
for ipoint,point in enumerate(data['points']):
    sector = [point]
    for iwall,wall in enumerate(data['walls']):
        if ipoint in wall:
            nex=None
            if wall[0]==ipoint:
                nex = data['points'][wall[1]]
            elif wall[1]==ipoint:
                nex = data['points'][wall[0]]
            length = distance(point, nex)
            vec = Vector([nex[i]-point[i] for i in range(3)]).unit()
            half = Point([point[i] + 0.5 * length * vec[i] for i in range(3)])
            sector.append([half, thick])
    all_tree_nodes.append(TreeNode([sector[0]] + [ secp[0] for secp in sector[1:]]))





for itreenode, treenode in enumerate(all_tree_nodes): 
    #thck = all_thicknesses[itreenode]
    #treenode.set_thicknesses()
    pl = treenode.offset(default_thickness = 0.2)
    polylines.append(pl)


import matplotlib.pyplot as plt

for pol in polylines:
    plt.plot([p[0] for p in pol], [p[1] for p in pol])

plt.show()



write_polylines_sets('simple_house', walls = polylines)
