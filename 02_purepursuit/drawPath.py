import matplotlib.pyplot as plt
import keyboard

fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_xlim([0, 9.144])
ax.set_ylim([0, 4.572])

#X_START = .762
#Y_START = .762
X_START = 1.143
Y_START = 2.16

points = [[X_START, Y_START]]

def onclick(event):
    plt.gcf()
    plt.ginput(1000)
    print('x=%f, y=%f'%(event.xdata, event.ydata))
    points.append([event.xdata, event.ydata])
    writeFile()
    fig.canvas.draw()

def writeFile():
    f = open("path.txt", "w")
    f.write(str(points))
    f.close()

cid = fig.canvas.mpl_connect('button_press_event', onclick)


X_START = .762
Y_START = .762
#X_START = 1.143
#Y_START = 2.16
# Plot the points on a graph
axis = [0, 9.144, 0, 4.572]
img = plt.imread("img.jpg")
plt.axis(axis)
plt.imshow(img, extent=axis)
plt.show()
