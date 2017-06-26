#!/usr/bin/env python

# import the necessary packages
import numpy as np
import sys
import time
import platform
import cv2
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import copy
from matplotlib import pyplot as plt
import threading

# helper dicts and variables
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

pieces_color = {
    0: (255, 0, 0),
    1: (0, 255, 255),
    2: (255, 0, 255),
    3: (255, 255, 0),
    4: (0, 0, 255),
    5: (0, 255, 0),
}

do_spin = False
cam_connected = False
cam = None
pub_board = None
pub_image = None
pub_sig = None
rot = 1  # camera is 0 ( 1 2 ..), 1 (-90 clockwise), 2 (+180), 3 (+90 clockwise)
entered = 0  # how many times did it try to find figures


class Board:
    def __init__(self):
        self.arr = [i % 2 for i in range(0, 64)]

    def print_board(self):
        for i in range(8):
            for j in range(8):
                print "{}\t".format(print_pieces[self.arr[i*8+j]]),
            print ""

    def __str__(self):
        s = ""
        # if self.rot == 0:
        for x in self.arr:
            s += print_pieces[x]
        # elif self.rot == 1:
        #     for x in range()
        # elif self.rot == 2:
        #     for x in range(63, -1, -1):
        #         s += print_pieces[x]
        # elif self.rot == 3:
        #     pass
        return s


def rectify(h):
    try:
        h = h.reshape((4, 2))
    except:
        return None
    h_new = np.zeros((4, 2), dtype=np.float32)

    add = h.sum(1)
    h_new[0] = h[np.argmin(add)]
    h_new[2] = h[np.argmax(add)]

    diff = np.diff(h, axis=1)
    h_new[1] = h[np.argmin(diff)]
    h_new[3] = h[np.argmax(diff)]
    return np.roll(h_new, 2)


def find_pieces(img):  # must be a square image
    sz = img.shape[0]
    start_x = int((700/16.)/700*sz)
    stop_x = int(670./700*sz)
    start_y = int((700/16.)/700*sz)
    stop_y = int(670./700*sz)
    move_x = int((700/8.)/700*sz) + 1
    move_y = int((700/8.)/700*sz) + 1
    half_len = int(15./700*sz)
    move_mid = 0  # int(10./700*sz)
    aku_limit = 100
    var_limit_w = 100
    var_limit_b = 34
    new_board = Board()
    count = 0
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # cv2.imshow("OPA", img_hsv)
    for i in range(start_x, stop_x, move_x):
        for j in range(start_y, stop_y, move_y):
            z_hsv = img_hsv[i-half_len:i+half_len,
                    j + (move_mid, 0)[j < stop_y / 2] - half_len:j + (move_mid, 0)[j < stop_y / 2] + half_len]
            z_bgr = img[i-half_len:i+half_len,
                    j + (move_mid, 0)[j < stop_y / 2] - half_len:j + (move_mid, 0)[j < stop_y / 2] + half_len]
            aku_hsv = np.median(z_hsv, axis=(0, 1))
            print "THIS AKU: ", i/move_x, " ",  j/move_y, " ", aku_hsv
            aku_bgr = np.mean(z_bgr, axis=(0, 1))
            aku_gray = aku_bgr[0] * 0.0722 + aku_bgr[1] * 0.7152 + aku_bgr[2] * 0.2126
            var = np.var(z_bgr, axis=(0, 1))
            var = sum(var)
            print var,
            print "HSV AKU:", aku_hsv
            h, s, v = aku_hsv
            if aku_gray > aku_limit:
                if s < 40:
                    new_board.arr[count] = pieces['white_b']
                elif var < var_limit_w:
                    new_board.arr[count] = pieces['white_k']
                else:
                    new_board.arr[count] = pieces['white_m']
            else:
                if v > 50:
                    new_board.arr[count] = pieces['black_b']
                elif var < var_limit_b:
                    new_board.arr[count] = pieces['black_k']
                else:
                    new_board.arr[count] = pieces['black_m']

            count += 1
        print ""

    count = 0
    for i in range(start_x, stop_x, move_x):
        for j in range(start_y, stop_y, move_y):
            img = cv2.putText(img, print_pieces[new_board.arr[count]], (j, i), cv2.FONT_HERSHEY_COMPLEX, 2, (0, 0, 255))
            cv2.circle(img, (j + (move_mid, 0)[j < stop_x / 2], i), half_len,
                       pieces_color[new_board.arr[count]], -1)
            count += 1

    # cv2.imshow("Figures found", img)

    return (new_board, img)


def get_transform(image):

    DRAW_CONTOURS = True

    image = cv2.GaussianBlur(image, (5, 5), 0)
    # cv2.imshow("Image to find board", image)

    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([89, 65, 29])
    upper_blue = np.array([140, 255, 255])
    filtered_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)

    # cv2.imshow("filtered blue 22", hsv_image)
    # cv2.imshow("filtered blue", filtered_blue)

    kernel = np.ones((3, 3), np.uint8)
    expanded_fblue = cv2.morphologyEx(filtered_blue, cv2.MORPH_DILATE, kernel, iterations=3)
    expanded_fblue = cv2.morphologyEx(expanded_fblue, cv2.MORPH_ERODE, kernel, iterations=4)
    expanded_fblue = cv2.morphologyEx(expanded_fblue, cv2.MORPH_DILATE, kernel, iterations=1)

    # cv2.imshow("Cleaned blue", expanded_fblue)

    im2, contours, hierarchy = cv2.findContours(expanded_fblue, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    if DRAW_CONTOURS:
        print(len(contours))
        for c in contours:
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.05 * peri, True)
            cv2.drawContours(image, [approx], -1, (0, 255, 0), 4)

    c = max(contours, key=cv2.contourArea)

    # print (contours.index(c))
    # contours.pop(contours.index(c))
    curr_max = 0
    real_max = cv2.contourArea(c)
    c2 = c
    for x in contours:
        ar = cv2.contourArea(x)
        if ar < real_max - 0.001 and curr_max < ar:
            curr_max = ar
            c2 = x

    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.05 * peri, True)
    if DRAW_CONTOURS:
        cv2.drawContours(image, [approx], -1, (0, 0, 255), 4)

    peri2 = cv2.arcLength(c2, True)
    approx2 = cv2.approxPolyDP(c2, 0.05 * peri2, True)
    if DRAW_CONTOURS:
        cv2.drawContours(image, [approx2], -1, (0, 0, 255), 4)
    # cv2.imshow("Im32", image)

    # cv2.waitKey(0)  # & 0xFF
    #
    # if cam_connected:
    #     cam.release()
    #
    # cv2.destroyAllWindows()

    # go crazy
    print (approx)
    if len(approx) > 4:
        approx = approx[0:4]
        print "Probably a bad picture"

    print len(approx)

    approx = rectify(approx)
    approx2 = rectify(approx2)
    h = np.array([[0, 0], [699, 0], [699, 699], [0, 699]], np.float32)
    retval = cv2.getPerspectiveTransform(approx, h)
    retval2 = cv2.getPerspectiveTransform(approx2, h)
    return (retval, retval2)


def get_transform_canny(image):
    # cv2.imshow("Image", image)

    kernel_sharpen_2 = np.array([[1, 1, 1], [1, -7, 1], [1, 1, 1]])
    output_2 = cv2.bilateralFilter(image, 9, 75, 75)

    edges = cv2.Canny(output_2, 42, 150)
    # cv2.imshow("Im2221", edges)
    kernel = np.ones((5, 5), np.uint8)
    # edgesD = cv2.erode(edges, kernel, iterations=1)
    edgesD = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)
    # cv2.imshow("Im2242", edgesD)

    # cv2.imshow("Im222", edges)
    # cv2.imshow("Im2223", edgesD)

    edges = edgesD
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
    # cv2.imshow("Im2", image)

    # go crazy
    if len(approx) > 4:
        approx = approx[0:4]

    print len(approx)

    approx = rectify(approx)
    h = np.array([[0, 0], [699, 0], [699, 699], [0, 699]], np.float32)
    retval = cv2.getPerspectiveTransform(approx, h)
    return (retval, retval)


def get_image():
    global cam_connected, cam

    if cam_connected:
        image = []
    else:
        image = cv2.imread("../examples/Boards/" + sys.argv[1])
        image = cv2.resize(image, None, fx=0.7, fy=0.7, interpolation=cv2.INTER_CUBIC)
        print "WHY?"

    if sys.argv[1] == 'cam':
        # clear buffer
        for _ in range(6):
            ret, frame = cam.read()
        # cv2.imshow('Camera frame', frame)
        image = frame
        image = cv2.resize(image, None, fx=0.7, fy=0.7, interpolation=cv2.INTER_CUBIC)

    return image


def check_ok(x):
    if x.count('.') != 32:
        return False
    return True


def spinner():
    e1 = cv2.getTickCount()

    global pub_board, pub_image, do_spin, pub_sig, entered
    rate = rospy.Rate(20)

    image = get_image()
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    l, a, b = cv2.split(cv2.cvtColor(image, cv2.COLOR_BGR2Lab))
    l_ch = clahe.apply(l)
    cv2.merge([l_ch, a, b], image)
    image = cv2.cvtColor(image, cv2.COLOR_Lab2BGR)
    bigger, smaller = get_transform(image)

    br = CvBridge()

    # do the figure detection
    image = get_image()
    warp = cv2.warpPerspective(image, smaller, (700, 700))

    frame_msg = br.cv2_to_imgmsg(warp, "bgr8")


    # cv2.imshow("Nova", warp)

    # identify pieces
    # z = find_pieces(warp)
    # p = z['board']
    # img = z['image']
    p, img = find_pieces(warp)
    p.print_board()
    if check_ok(str(p)):
        entered = 0
        pub_board.publish(String(str(p)))
        print str(p)
        pub_image.publish(br.cv2_to_imgmsg(img, "bgr8"))
        pub_sig.publish(String("CAMERA_FIN"))
    else:
        if entered > 30:
            print "Failed to take the picture"

        print("There was an error in the camera algorithm")
        pub_sig.publish(String("CAMERA_GO"))
        entered += 1

    e2 = cv2.getTickCount()
    time_past = (e2 - e1) / cv2.getTickFrequency()
    print time_past
    # spin with 20 Hz
    rate.sleep()


def camera_sig_callback(msg):
    global entered
    if msg.data == "CAMERA_GO":
        entered = 0
        spinner()
    elif msg.data == "CAMERA_STOP":
        pass


def main():
    global cam, cam_connected, pub_board, pub_image, pub_sig

    rospy.init_node('checkers_camera')

    rospy.Subscriber('/checkers/camera_sig', String, camera_sig_callback, queue_size=50)
    pub_board = rospy.Publisher('/checkers/board_msg', String, queue_size=50)
    pub_image = rospy.Publisher('/checkers/image_msg', Image, queue_size=50)
    pub_sig = rospy.Publisher('/checkers/camera_sig', String, queue_size=50)

    if sys.argv[1] == 'cam':
        cam_connected = True
        cam = cv2.VideoCapture(0)
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1000)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1000)

    time.sleep(0.02)  # wait for node and camera to stabilise

    # t = threading.Thread(target=spinner)
    # t.start()
    spinner()

    # check time elapsed



    # wait for key
    # cv2.waitKey(0)  # & 0xFF
    rospy.spin()

    print "Exiting camera node"

    if cam_connected:
        cam.release()

    # cv2.destroyAllWindows()
    return

if __name__ == "__main__":
    main()
