# This is a sample Python script.

# Press Maj+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import matplotlib.pyplot as plt
import matplotlib
import numpy as np

matplotlib.use('TkAgg')


def plotpolar(filename):
    f = open(filename, 'r')
    fichier = f.readlines()

    angles = []
    dist = []

    for j in range(1, len(fichier)):
        tempo = fichier[j].split()
        if (float(tempo[1])<1.0):
            angles.append(float(tempo[0])*2*np.pi/360)
            dist.append(float(tempo[1]))
    plt.polar(angles, dist, ".", color ="green")
    plt.show()


def plotXY(filename):
    f = open(filename, 'r')
    fichier = f.readlines()

    x = []
    y = []

    for j in range(0, len(fichier)):
        tempo = fichier[j].split()
        x.append(float(tempo[0]))
        y.append(float(tempo[1]))
    plt.axis('equal')
    plt.plot([-1, 1, 1, -1, -1], [1.5, 1.5, -1.5, -1.5, 1.5], color="black")
    plt.xlim([-1.7,1.7])
    plt.plot(x, y, ".", color="green", label="position with Lidar")
    plt.title("Lidar position")
    plt.show()

def plotAngle(filename):
    f = open(filename, 'r')
    fichier = f.readlines()

    a = []
    for j in range(0, len(fichier)):
        tempo = fichier[j].split()
        a.append(float(tempo[0]))
    plt.ylim([-180,180])
    plt.gca().get_xaxis().set_visible(False)
    plt.plot(a, ".", color="green", label="position with Lidar")
    plt.title("Lidar orientation")
    plt.show()



if __name__ == '__main__':
    plotpolar("PlotPy/testBottom.txt")

