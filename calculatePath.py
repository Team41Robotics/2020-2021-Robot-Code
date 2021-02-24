import math
import matplotlib.pyplot as plt
import pandas as pd

pts = [[0, 0], [1, 1], [2, 1], [3, 2], [4, 4], [5, 3], [5.2, 2]]
SPACING = 0.1524
newPoints = []
segments = [[pts[i-1], pts[i]] for i in range(1, len(pts))]

def smooth(path, b, tolerance):
    a = 1.0-b
    smoothPath = path.copy()

    change = tolerance
    while change >= tolerance:
        change = 0.0
        for i in range(1, len(path)- 1):
            for j in range(0, len(path[i])):
                aux = smoothPath[i][j]
                smoothPath[i][j] += a * (path[i][j] - smoothPath[i][j]) + b * (smoothPath[i-1][j] + smoothPath[i+1][j] - (2.0 * smoothPath[i][j]))
                change += abs(aux - smoothPath[i][j])
    
    return smoothPath                            
    
def curvature(path):
    ret = []
    ret.append(0)

    for i in range(1, len(path)-1):
        x1 = path[i-1][0]
        x2 = path[i][0]
        x3 = path[i+1][0]
        y1 = path[i-1][1]
        y2 = path[i][1]
        y3 = path[i+1][1]

        k1 = 0.5 * (x1**2 + y1**2 - x2**2 - y2**2) / (x1-x2)
        k2 = (y1-y2)/(x1-x2)
        b = 0.5 * (x2**2-(2*x2*k1) + y2**2-x3**2+(2*x3*k1)-y3**2) / ((x3*k2)-y3+y2-(x2*k2))
        a = k1-k2*b
        r = math.sqrt((x1-a)**2 + (y1-b)**2)
        ret.append(1/r)
    
    ret.append(0)
    return ret

VELOCITY_MAX = 5
K_Vel = 1 # Apparently should be set between 1-5
def velocity(path,curvatures):
    # Find initial velocities
    vel = []
    for c in curvatures:
        if c == 0:
            vel.append(0)
        else:
            vel.append(math.min(VELOCITY_MAX, K_Vel/c))
    
    # smooth velocities with max acceleration
    vel(len(arr)-1) = 0    
    for i in reversed(xrange(len(vel)-1)):
        if i != 0:
            a = 1 # acceleration max
            x1 = 
            x0 = 
            y1 = 
            y0 = 
            d = math.sqrt(path[i]) # distance between the points
            vel(i) = math.sqrt(vel(i-1)**2 + 2*a*d)

        


for seg in segments:
    vector = [(seg[1][0] - seg[0][0]), (seg[1][1] - seg[0][1])]
    magn = math.sqrt(vector[0]**2+vector[1]**2)
    norm = [vector[0]/magn, vector[1]/magn]
    num_points_that_fit = math.ceil(magn/SPACING)
    vector = [norm[0]*SPACING, norm[1]*SPACING]
    for i in range(0, num_points_that_fit):
        newPoints.append([seg[0][0]+(vector[0]*i), seg[0][1]+(vector[1]*i)])

newPoints = smooth(newPoints, 0.0025, .001)

img = plt.imread("img.png")
plt.plot([pt[0] for pt in newPoints], [pt[1] for pt in newPoints], "bo")
plt.axis([0, 9.144, 0, 4.572])
plt.show()
