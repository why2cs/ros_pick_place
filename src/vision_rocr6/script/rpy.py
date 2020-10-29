import numpy
import math

def foo(matrix):
    Rr, Rp, Ry = None, None, None
    if abs(abs(matrix[0,2])-1)<1e-6:
        Rr=0
        if matrix[0,2]>0:
            Ry=math.atan2(matrix[2,1],matrix[1,1])
        else:
            Ry=-math.atan2(matrix[1,0],matrix[2,0])
        Rp=math.asin(matrix[0,2])
    else:
        Rr=-math.atan2(matrix[0,1],matrix[0,0])
        Ry=-math.atan2(matrix[1,2],matrix[2,2])
        Rp=math.atan(matrix[0,2]*math.cos(Rr)/matrix[0,0])
    return (Rr, Rp, Ry)



m = numpy.array([
[-1.904579860287263e-08, -0.7033946933510905, -0.710799483233862, 0.03384799006603334],
[0.9999999999999997, 1.770842271775634e-17, -2.679489654195813e-08, -0.10600000179525804],
[1.8847388049091973e-08, -0.7107994832338622, 0.7033946933510903, 0.7721567456555924],
[0.0, 0.0, 0.0, 1.0]
])

print foo(m)

