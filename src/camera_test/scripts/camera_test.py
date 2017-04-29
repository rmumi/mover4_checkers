#!/usr/bin/env python

# import the necessary packages
import numpy as np
import cv2
from matplotlib import pyplot as plt

pieces = {
    'white_b': 0,
    'black_b': 1,
    'white_m': 2,
    'black_m': 3,
    'white_k': 4,
    'black_k': 5,
}

print_pieces = {
    0: '.',
    1: '_',
    2: 'w',
    3: 'b',
    4: 'W',
    5: 'B',
}


class Board:

    def __init__(self):
        self.arr = [i%2 for i in range(0, 64)]

    def print_board(self):
        for i in range(8):
            for j in range(8):
                print "{}\t".format(print_pieces[self.arr[i*8+j]]),
            print ""


def nothing(x):
    pass


def rectify(h):
    h = h.reshape((4, 2))
    hnew = np.zeros((4, 2), dtype=np.float32)

    add = h.sum(1)
    hnew[0] = h[np.argmin(add)]
    hnew[2] = h[np.argmax(add)]

    diff = np.diff(h, axis=1)
    hnew[1] = h[np.argmin(diff)]
    hnew[3] = h[np.argmax(diff)]

    return hnew


def find_pieces(img):  # must be square image
    #img = cv2.resize(img, (0, 0), fx=0.7, fy=0.7)
    sz = img.shape[0]
    start_x = int(79./700*sz)
    stop_x = int(670./700*sz)
    start_y = int(79./700*sz)
    stop_y = int(670./700*sz)
    move_x = int(77.5/700*sz)
    move_y = int(77.5/700*sz)
    half_len = int(17./700*sz)
    move_mid = int(10./700*sz)
    aku_limit = 128
    var_limit = 600
    new_board = Board()
    count = 0

    for i in range(start_x, stop_x, move_x):
        for j in range(start_y, stop_y, move_y):
            z = img[i-half_len:i+half_len,
                    j + (move_mid, 0)[j < stop_y / 2] - half_len:j + (move_mid, 0)[j < stop_y / 2] + half_len]
            aku = np.mean(z, axis=(0, 1))
            aku = aku[0] * 0.0722 + aku[1] * 0.7152 + aku[2] * 0.2126
            var = np.var(z, axis=(0, 1))
            var = sum(var)
            print var

            if aku > aku_limit:
                if var < var_limit:
                    new_board.arr[count] = pieces['white_b']
                else:
                    new_board.arr[count] = pieces['white_m']
            else:
                if var < var_limit:
                    new_board.arr[count] = pieces['black_b']
                else:
                    new_board.arr[count] = pieces['black_m']

            count += 1

    count = 0
    for i in range(start_x, stop_x, move_x):
        for j in range(start_y, stop_y, move_y):
            cv2.circle(img, (j + (move_mid, 0)[j < stop_x / 2], i), half_len,
                       (0 if new_board.arr[count] < 2 else 255, 255 if new_board.arr[count]&1 else 0, 255), -1)
            count += 1

    cv2.imshow("WATY", img)

    return new_board


def main():
    # load the games image
    e1 = cv2.getTickCount()
    image = cv2.imread("/home/rijad/Pictures/Webcam/image6.jpg")
    # img = cv2.medianBlur(image, 5)
    # img = cv2.bilateralFilter(image, 9, 75, 75)
    image = cv2.resize(image, None, fx=0.7, fy=0.7, interpolation=cv2.INTER_CUBIC)
    cv2.imshow("Image", image)
    # cv2.waitKey(1000)

    # plt.imshow(edges, cmap='gray', interpolation='bicubic')  # cv2.merge([r, g, b]) # img2 = img[:,:,::-1]
    # plt.show()
    # cv2.namedWindow('Image')
    # cv2.createTrackbar('Min', 'Image', 0, 255, nothing)
    # cv2.createTrackbar('Max', 'Image', 0, 255, nothing)
    # while 1:
    #     k = cv2.waitKey(2) & 0xFF
    #     if k == 27:
    #         break
    #     x = cv2.getTrackbarPos('Min', 'Image')
    #     y = cv2.getTrackbarPos('Max', 'Image')
    #     upper = np.array([y, y, y])
    #     lower = np.array([x, x, x])
    #     mask = cv2.inRange(image, lower, upper)
    #     cv2.imshow("Image", mask)
    # cv2.waitKey(13000)
    # cv2.namedWindow('Image4')
    # cv2.createTrackbar('Min', 'Image4', 0, 255, nothing)
    # cv2.createTrackbar('Max', 'Image4', 0, 255, nothing)

    kernel_sharpen_2 = np.array([[1, 1, 1], [1, -7, 1], [1, 1, 1]])
    output_2 = cv2.bilateralFilter(image, 9, 75, 75)
    output_2 = cv2.filter2D(output_2, -1, kernel_sharpen_2)

    edges = cv2.Canny(output_2, 42, 170)
    kernel = np.ones((5, 5), np.uint8)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)
    # edgesD = cv2.dilate(edges, kernel, iterations=1)
    # gradient = cv2.morphologyEx(edges, cv2.MORPH_GRADIENT, kernel)
    cv2.imshow("Im2", edges)

    im2, contours, hierarchy = cv2.findContours(edges, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    # print(len(contours))
    # for c in contours:
    #     peri = cv2.arcLength(c, True)
    #     approx = cv2.approxPolyDP(c, 0.05 * peri, True)
    #     cv2.drawContours(image, [approx], -1, (0, 255, 0), 4)

    c = max(contours, key=cv2.contourArea)
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.05 * peri, True)
    cv2.drawContours(image, [approx], -1, (0, 0, 255), 4)
    cv2.imshow("Im2", image)

    approx = rectify(approx)
    h = np.array([[0, 0], [699, 0], [699, 699], [0, 699]], np.float32)
    retval = cv2.getPerspectiveTransform(approx, h)
    warp = cv2.warpPerspective(image, retval, (700, 700))
    cv2.imshow("Nova", warp)

    zwarp = warp
    warp = cv2.cvtColor(warp, cv2.COLOR_BGR2GRAY)
    cv2.imshow("Nesto", warp)
    mask = cv2.cvtColor(zwarp, cv2.COLOR_BGR2GRAY)
    x = 230 * 2 + 3  # 230 cv2.getTrackbarPos('Min', 'Image4')
    y = 35  # 58 cv2.getTrackbarPos('Max', 'Image4')
    maska = cv2.adaptiveThreshold(mask, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, x, y)
    cv2.imshow("Image4", maska)

    lab = cv2.cvtColor(zwarp, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    # cv2.imshow('l_chh', l)
    # cv2.imshow('a_chh', a)
    # cv2.imshow('b_chh', b)
    clahe = cv2.createCLAHE(clipLimit=2, tileGridSize=(8, 8))
    cl = clahe.apply(l)
    cv2.imshow('CLAHE, output', cl)
    limg = cv2.merge((cl, a, b))
    final = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
    cv2.imshow('Final', final)
    final = cv2.bilateralFilter(final, 7, 75, 75)

    find_pieces(final).print_board()


    warp = cv2.cvtColor(zwarp, cv2.COLOR_BGR2GRAY)
    # ewarp = cv2.Canny(warp, 42, 170)
    circles = cv2.HoughCircles(warp, method=cv2.HOUGH_GRADIENT, dp=2.1, minDist=30, minRadius=20, maxRadius=60)

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")

        for (x, y, r) in circles:
            cv2.circle(zwarp, (x, y), r, (0, 255, 0), 4)

    # cv2.imshow("Slika", zwarp)

    # b, g, r = cv2.split(zwarp)
    # b.fill(0)
    # g.fill(0)
    # zwarp = cv2.merge((b, g, r))
    # cv2.imshow("SlikaX", zwarp)



    e2 = cv2.getTickCount()
    time = (e2 - e1) / cv2.getTickFrequency()
    print time

    cv2.waitKey(0)  # & 0xFF
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()