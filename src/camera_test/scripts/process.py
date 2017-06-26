import copy
from collections import deque
import sys
from time import sleep
import matplotlib
import pylab
import os
import matplotlib.pyplot as plt

import numpy as np
from math import sin, cos

a0 = 220
a1 = 190
a2 = 220
a3 = 48 + 95
PI = 3.14159265359
deg2rad = PI / 180.0
rad2deg = 180.0 / PI

DH_par = [
[0.0, a0, -PI/2, 0],
[-PI/2, 0, 0, a1],
[0.0, 0, 0, a2],
[0.0, 0, 0, a3],
]


def fkine(rb):
    A = np.matrix([[1., 0, 0, 0], [0, 1, 0., 0], [0, 0., 1, 0], [0, 0., 0, 1]])
    th = [rb[0] + DH_par[0][0],
          rb[1] + DH_par[1][0],
          rb[2] + DH_par[2][0],
          rb[3] + DH_par[3][0]]
    for i in range(4):
        A *= np.matrix([[cos(th[i]), -sin(th[i]), 0, 0],
                        [sin(th[i]), cos(th[i]), 0, 0],
                        [0, 0, 1, DH_par[i][1]],
                        [0, 0, 0, 1]])
        A *= np.matrix([[1, 0, 0, DH_par[i][3]],
                        [0, cos(DH_par[i][2]), -sin(DH_par[i][2]), 0],
                        [0, sin(DH_par[i][2]), cos(DH_par[i][2]), 0],
                        [0, 0, 0, 1]])
    # print A
    return A.A


def get_xyzphi(rb):
    return [round(x, 6) for x in [x for x in fkine(rb)[0:3, 3]] + [rb[1] + rb[2] + rb[3]]]



if __name__ == "__main__":
    # for filename in os.listdir("./recordings"):
    list_names = ["sP0.txt", "oP0.txt", "sPIDMM0.txt", "oPID0.txt", "oPID2.txt"]
    every_plot = {}
    for filename in list_names:
        # filename =
        f = open("./recordings/" + filename, 'r')
        all_moments = []
        all_velocities = []
        all_real_q = []
        all_req_q = []
        all_real_xyzphi = []
        all_req_xyzphi = []
        for line in f:
            if len(line) < 4:
                continue
            moments, real_q, req_q = line.split('/')
            velocities = [float(x.split(';')[1])-127 for x in moments.split(':')]
            moments = [float(x.split(';')[4]) for x in moments.split(':')]
            real_q = [float(x)*deg2rad for x in real_q.split(';')[0:-1]]
            req_q = [float(x)*deg2rad for x in req_q.split(';')[0:-1]]
            real_xyzphi = get_xyzphi(real_q)
            req_xyzphi = get_xyzphi(req_q)
            all_moments.append(moments)
            all_velocities.append(velocities)
            all_real_q.append(real_q)
            all_req_q.append(req_q)
            all_real_xyzphi.append(real_xyzphi)
            all_req_xyzphi.append(req_xyzphi)
        f.close()
        print len(all_velocities)
        labels = ['Zglob 1', 'Zglob 2', 'Zglob 3', 'Zglob 4']
        line_types = ['b-', 'r-', 'g-', 'y-']
        every_plot[filename + "v"] = plt.figure()
        for i in range(4):
            plt.plot(np.arange(0, 18, 18./len(all_velocities)), (np.matrix(all_velocities)[:, i]/(10 if i == 3 else 1)).A, line_types[i], label=labels[i])
        plt.ylim([-30, 30])
        plt.legend(loc='best')
        plt.title("Brzine " + filename)
        plt.grid()

        labels = ['Zglob 1', 'Zglob 2', 'Zglob 3', 'Zglob 4']
        line_types = ['b-', 'r-', 'g-', 'y-']
        every_plot[filename + "m"] = plt.figure()
        for i in range(4):
            plt.plot(np.arange(0, 18, 18. / len(all_velocities)),
                     (np.matrix(all_moments))[:, i].A, line_types[i], label=labels[i])
        plt.ylim([0, 80])
        plt.legend(loc='best')
        plt.title("Momenti " + filename)
        plt.grid()

    for z in every_plot:
        every_plot[z].show()

    raw_input()

