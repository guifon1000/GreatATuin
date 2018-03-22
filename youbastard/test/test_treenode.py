
from youbastard.geometry.TreeNode import TreeNode
from youbastard.geometry.Point import Point
from youbastard.geometry.Vector import Vector
from youbastard.geo_functions import distance
import numpy as np
import matplotlib.pyplot as plt
import time

r1 = 1.
r2 = 1.
teta1 = 0.2
teta2 = 0.5
data = {'points': [Point([0., 0., 0.]), 
            Point([r1 * np.cos(teta1), r1 * np.sin(teta1), 0.]), 
            Point([1., 1., 0.]), 
            Point([r2 * np.cos(teta2), r2 * np.sin(teta2), 0.])], 
        'walls': [[0,1],[0,2], [0,3]]}
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
    pl = treenode.offset(default_thickness = 0.02)
    polylines.append(pl)
for pl in polylines:
    plt.plot([p[0] for p in pl],[p[1] for p in pl])
    plt.show()
        
        

