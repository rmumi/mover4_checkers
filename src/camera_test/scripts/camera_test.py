#!/usr/bin/env python

# import the necessary packages
import numpy as np
import cv2
from matplotlib import pyplot as plt


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


# load the games image
e1 = cv2.getTickCount()
image = cv2.imread("/home/rijad/Pictures/Webcam/image8.jpg")
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

circles = cv2.HoughCircles(warp, method=cv2.HOUGH_GRADIENT, dp=2.1, minDist=30, minRadius=20, maxRadius=60)
if circles is not None:
    circles = np.round(circles[0, :]).astype("int")

    for (x, y, r) in circles:
        cv2.circle(zwarp, (x, y), r, (0, 255, 0), 4)

cv2.imshow("Slika", zwarp)

e2 = cv2.getTickCount()
time = (e2 - e1) / cv2.getTickFrequency()
print time


cv2.waitKey(0)  # & 0xFF
cv2.destroyAllWindows()
















# tested before
# !/usr/bin/env python

# import the necessary packages
# import numpy as np
# import cv2
# from matplotlib import pyplot as plt
#
# # load the games image
# image = cv2.imread("/home/rijad/Pictures/Webcam/image8.jpg")
# cv2.imshow("Image", image)
# cv2.waitKey(1000)
#
# # find the red color game in the image
# upper = np.array([75, 75, 75])
# lower = np.array([0, 0, 0])
# mask = cv2.inRange(image, lower, upper)
# cv2.imshow("Image2", mask)
# cv2.waitKey(1000)
# edges = cv2.Canny(mask, 100, 200)
# cv2.imshow("Image3", edges)
# # plt.imshow(edges, cmap='gray', interpolation='bicubic')  # cv2.merge([r, g, b]) # img2 = img[:,:,::-1]
# # plt.show()
# cv2.waitKey(13000)
#
# # find contours in the masked image and keep the largest one
# (_, cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# c = max(cnts, key=cv2.contourArea)
#
# # approximate the contour
# peri = cv2.arcLength(c, True)
# approx = cv2.approxPolyDP(c, 0.05 * peri, True)
#
# # draw a green bounding box surrounding the red game
# cv2.drawContours(image, [approx], -1, (0, 255, 0), 4)
# cv2.imshow("Image", image)
# cv2.waitKey(10000)  # & 0xFF
# cv2.destroyAllWindows()
