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
        for x in self.arr:
            s += print_pieces[x]
        return s


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
    start_x = int((700/16.)/700*sz)
    stop_x = int(670./700*sz)
    start_y = int((700/16.)/700*sz)
    stop_y = int(670./700*sz)
    move_x = int((700/8.)/700*sz)
    move_y = int((700/8.)/700*sz)
    half_len = int(15./700*sz)
    move_mid = 0#int(10./700*sz)
    aku_limit = 100
    var_limit_w = 200
    var_limit_b = 40
    new_board = Board()
    count = 0

    for i in range(start_x, stop_x, move_x):
        for j in range(start_y, stop_y, move_y):
            z = img[i-half_len:i+half_len,
                    j + (move_mid, 0)[j < stop_y / 2] - half_len:j + (move_mid, 0)[j < stop_y / 2] + half_len]
            aku = np.mean(z, axis=(0, 1))
            aku_gray = aku[0] * 0.0722 + aku[1] * 0.7152 + aku[2] * 0.2126
            var = np.var(z, axis=(0, 1))
            var = sum(var)
            print var,
            b, g, r = aku
            if aku_gray > aku_limit:
                if r/b < 1.2 and b/g < 1.2:
                    new_board.arr[count] = pieces['white_b']
                elif var < var_limit_w:
                    new_board.arr[count] = pieces['white_k']
                else:
                    new_board.arr[count] = pieces['white_m']
            else:
                if r / b > 1.5 and r / g > 1.5:
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

    cv2.imshow("Figures found", img)

    return new_board


def get_transform(image):

    DRAW_CONTOURS = False

    cv2.imshow("Image to find board", image)

    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([100, 100, 70])
    upper_blue = np.array([140, 255, 255])
    filtered_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)

    kernel = np.ones((3, 3), np.uint8)
    expanded_fblue = cv2.morphologyEx(filtered_blue, cv2.MORPH_DILATE, kernel, iterations=9)
    expanded_fblue = cv2.morphologyEx(expanded_fblue, cv2.MORPH_ERODE, kernel, iterations=11)
    expanded_fblue = cv2.morphologyEx(expanded_fblue, cv2.MORPH_DILATE, kernel, iterations=2)

    cv2.imshow("Cleaned blue", expanded_fblue)

    im2, contours, hierarchy = cv2.findContours(expanded_fblue, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    if DRAW_CONTOURS:
        print(len(contours))
        for c in contours:
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.05 * peri, True)
            cv2.drawContours(image, [approx], -1, (0, 255, 0), 4)

    c = max(contours, key=cv2.contourArea)

    contours.remove(c)

    c2 = max(contours, key=cv2.contourArea)

    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.05 * peri, True)
    if DRAW_CONTOURS:
        cv2.drawContours(image, [approx], -1, (0, 0, 255), 4)

    peri2 = cv2.arcLength(c2, True)
    approx2 = cv2.approxPolyDP(c2, 0.05 * peri, True)
    if DRAW_CONTOURS:
        cv2.drawContours(image, [approx2], -1, (0, 0, 255), 4)
    #cv2.imshow("Im2", image)

    # go crazy
    if len(approx) > 4:
        approx = approx[0:4]
        print "PROBABLY A BAD PICTURE"

    print len(approx)

    approx = rectify(approx)
    approx2 = rectify(approx2)
    h = np.array([[0, 0], [699, 0], [699, 699], [0, 699]], np.float32)
    retval = cv2.getPerspectiveTransform(approx, h)
    retval2 = cv2.getPerspectiveTransform(approx2, h)
    return (retval, retval2)


def get_image():
    global cam_connected, cam

    if cam_connected:
        image = []
    else:
        image = cv2.imread("../examples/Boards/" + sys.argv[1])
        image = cv2.resize(image, None, fx=0.7, fy=0.7, interpolation=cv2.INTER_CUBIC)

    if sys.argv[1] == 'cam':
        ret, frame = cam.read()
        cv2.imshow('Camera frame', frame)
        image = frame
        image = cv2.resize(image, None, fx=0.7, fy=0.7, interpolation=cv2.INTER_CUBIC)

    return image


def spinner():
    global pub_board, pub_image, do_spin
    rate = rospy.Rate(20)

    image = get_image()
    bigger, smaller = get_transform(image)

    br = CvBridge()

    while True:
        image = get_image()
        warp = cv2.warpPerspective(image, smaller, (700, 700))

        frame_msg = br.cv2_to_imgmsg(warp, "rgb8")
        pub_image.publish(frame_msg)

        # cv2.imshow("Nova", warp)

        # identify pieces
        p = find_pieces(warp)
        p.print_board()
        pub_board.publish(String(str(p)))
        print p
        if not do_spin:
            break
        # spin with 20 Hz
        rate.sleep()


def camera_sig_callback(msg):
    if msg.data == "CAMERA_GO":
        do_spin = True
    elif msg.data == "CAMERA_STOP":
        do_spin = False


def main():
    global cam, cam_connected, pub_board, pub_image

    rospy.init_node('checkers_camera')

    rospy.Subscriber('/checkers/camera_sig', String, camera_sig_callback, queue_size=50)
    pub_board = rospy.Publisher('/checkers/board_msg', String, queue_size=50)
    pub_image = rospy.Publisher('/checkers/image_mss', Image, queue_size=50)

    e1 = cv2.getTickCount()

    if sys.argv[1] == 'cam':
        cam_connected = True
        cam = cv2.VideoCapture(0)
        cam.set(3, 1000)
        cam.set(4, 1000)

    time.sleep(0.02)  # wait for node and camera to stabilise

    # t = threading.Thread(target=spinner)
    # t.start()
    spinner()

    # check time elapsed
    e2 = cv2.getTickCount()
    time_past = (e2 - e1) / cv2.getTickFrequency()
    print time_past

    # wait for key
    cv2.waitKey(0)  # & 0xFF

    if cam_connected:
        cam.release()

    cv2.destroyAllWindows()
    return

if __name__ == "__main__":
    main()
