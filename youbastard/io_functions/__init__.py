import json


def pretty_print(string, symbol = '-'):
    s = ''
    for i in range(79):
        s += symbol
    s += '\n'
    lstr = len(string) + 2 
    s2 = ''
    if lstr >= 79:
        s2 = string + '\n'
    else:
        si = ' ' + string + ' '
        n1 = int((79 - lstr )/2)
        n2 = 79 - len(si) - n1
        s2 = ''
        for i in range(n1): s2 += symbol
        s2 += si
        for i in range(n2): s2 += symbol 
        s2 += '\n'
    print( s + s2 + s)


def write_geo(name,geom):
    fg = open(name+'.geo','w')
    for l in geom.get_code():
        fg.write(l)
    fg.close()


def write_polylines_sets(name, **kwargs):
    json.dump(kwargs, open(name+'.json','w')) 

