def de_boor(control_points, parameter):
    p = len(control_points)
    U = [0. for i in range(p+1)] + [1. for i in range(p+1)]
    ar = control_points
    for r in range(p):
        for i in range(len(ar)):
            alpha = (parameter - U[i])/(U[i+1+p-r] - U[i]) 
            print 'iteration '+str(r)
            print alpha
            print '----'


    
