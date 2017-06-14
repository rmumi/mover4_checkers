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
    # auto p (A.GetPosVec());
    # auto R (A.GetRot());
    # robotState z;
    # for(int i = 0; i < 3; i++) z.p[i] = p[i];
    # z.p[3] = atan2(R[1][2], R[0][2]);
    # z.p[4] = atan2(sqrt(R[0][2]*R[0][2] + R[1][2]*R[1][2]), R[2][2]);
    # z.p[5] = atan2(R[2][1], -R[2][0]);
    # return z;

board_x_dr = 126+208.
board_y_dr = 104.
board_x_ul = 126.
board_y_ul = -104.
board_v_x = (1 / 14. * (+(board_x_ul - board_x_dr) - (board_y_ul - board_y_dr)),
                  1 / 14. * (+(board_x_ul - board_x_dr) + (board_y_ul - board_y_dr)))
board_v_y = (1 / 14. * (+(board_x_ul - board_x_dr) + (board_y_ul - board_y_dr)),
                  1 / 14. * (-(board_x_ul - board_x_dr) + (board_y_ul - board_y_dr)))
board_xy = [(board_x_dr + x * board_v_x[0] + y * board_v_y[0],
            board_y_dr + x * board_v_x[1] + y * board_v_y[1])
            for y, x in np.ndindex((8, 8))]


def compute_par(inputs, wished, given):
    z = []
    # z
    for x in inputs:
        z.append([1, cos(x[1]), cos(x[1] + x[2]), cos(x[1] + x[2] + x[3])])
    # x
    for x in inputs:
        z.append([(d) * cos(x[0])
                  for d in [0, sin(x[1]), sin(x[1] + x[2]), sin(x[1] + x[2] + x[3])]])
    # y
    for x in inputs:
        z.append([(d) * sin(x[0])
                  for d in [0, sin(x[1]), sin(x[1] + x[2]), sin(x[1] + x[2] + x[3])]])
    Z = np.matrix(z)
    return sum(sum((np.matrix([x*x for x in ((Z*given)-wished).flatten().A]).A)))
    # print (Z.T*Z).I*Z.T*wished,
    # print Z, wished
    

if __name__ == "__main__":

    f = open('pozicije2')
    u = []
    # print u
    i = 0
    for line in f:
        u.append([int(x)*deg2rad for x in line.split('|')[-1][0:-1].split(',')])
        i += 1
    board_xy = board_xy[1:64:2]
    o = [[50.]]*32
    o.extend([[x[0]] for x in reversed(board_xy)])
    o.extend([[x[1]] for x in reversed(board_xy)])
    # print o
    # print o
    minio = 111111111111
    ma = []
    for da0 in range(-80, 10, 2):
        for da1 in range(-5, 5):
            for da2 in range(-30, 5):
                for da3 in range(-5, 5):
                    l = compute_par(u, np.matrix(o), np.matrix([[a0+da0], [a1+da1],
                                                                [a2+da2], [a3+da3]]))
                    if l < minio:
                        ma = [[a0+da0], [a1+da1], [a2+da2], [a3+da3]]
                        minio = l
    print ma, minio
    # for x in u:
    #     print fkine(x)[2][3]

    f.close()
