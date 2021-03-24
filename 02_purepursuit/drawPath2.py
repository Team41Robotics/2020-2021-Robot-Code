import matplotlib.pyplot as plt

#X_START = .762
#Y_START = .762
X_START = 1.143
Y_START = 2.16

#pts = [[X_START, Y_START]]
pts = []
def writeFile():
    f = open("bounce4.txt", "w")
    f.write(str(pts))
    f.close()

class PointPlotter(object):
    def plotPoint(self):
        ax = plt.gca()
        xy = plt.ginput(1000, 5000)

        x = [p[0] for p in xy]
        y = [p[1] for p in xy]
        plt.plot(x,y)
        for i in range(0, len(x)):
            pts.append([x[i],y[i]])
        writeFile()

axis = [0, 9.144, 0, 4.572]
img = plt.imread("img3.png")
plt.axis(axis)
plt.imshow(img, extent=axis)

ld = PointPlotter()
ld.plotPoint()
