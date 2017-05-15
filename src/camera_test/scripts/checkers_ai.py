import rospy
from std_msgs.msg import String

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

last_board = "bbbbbbbbbbbb________wwwwwwwwwwww"

b_moves = {}
w_moves = {}
for x in range(1, 33):
    if ((x-1) / 4) % 2 == 0:
        if x % 4 == 0:
            if x+4 < 33:
                b_moves[x] = [x+4]
            if x-4 > 0:
                w_moves[x] = [x-4]
        else:
            if x+5 < 33:
                b_moves[x] = [x+4, x+5]
            if x-4 > 0:
                w_moves[x] = [x-4, x-3]
    else:
        if x % 4 == 1:
            if x+4 < 33:
                b_moves[x] = [x+4]
            if x-4 > 0:
                w_moves[x] = [x-4]
        else:
            if x+4 < 33:
                b_moves[x] = [x+3, x+4]
            if x-5 > 0:
                w_moves[x] = [x-5, x-4]


def signal_callback(msg):
    if msg.data == "AI_GO_WHITE":
        pub.publish("22-18")
    elif msg.data == "AI_GO_BLACK":
        pub.publish("1x10;10x17")


def board_callback(msg):

    pass


if __name__ == "__main__":
    rospy.init_node("checkers_ai")

    pub = rospy.Publisher("/checkers/moves_msg", String, queue_size=50)

    sub = rospy.Subscriber("/checkers/ai_sig", String, signal_callback, queue_size=50)

    sub_board = rospy.Subscriber("/checkers/board", String, board_callback, queue_size=50)

    rospy.spin()
