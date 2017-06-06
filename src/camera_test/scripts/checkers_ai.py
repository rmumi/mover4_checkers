from __future__ import print_function
import rospy
from std_msgs.msg import String
import copy
from collections import deque
import sys
from time import sleep


def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

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

INF = 100000000
max_depth = 7
pub_moves = None
pub_sig = None
init_board = "bbbbbbbbbbbb________wwwwwwwwwwww"
last_board = init_board
current_board = init_board

b_moves = {}
w_moves = {}
b_jumps = {}
w_jumps = {}


def calculate_all_moves():
    global b_moves, b_jumps, w_moves, w_jumps
    # standard moves
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
    # jumps
    for x in range(1, 33):
        if x % 4 == 0:  # 4x11
            if x+7 < 33:
                b_jumps[x] = [x+7]
            if x-9 > 0:
                w_jumps[x] = [x-9]
        elif x % 4 == 1:  # 1x10
            if x+9 < 33:
                b_jumps[x] = [x+9]
            if x-7 > 0:
                w_jumps[x] = [x-7]
        else:
            if x+9 < 33:  # 2x9 2x11
                b_jumps[x] = [x+7, x+9]
            if x-9 > 0:
                w_jumps[x] = [x-9, x-7]


def eval_board(board, white):
    score = 0  # positive for black winning
    for x in board:
        if x == 'b':
            score += 1
        elif x == 'B':
            score += 2
        elif x == 'w':
            score += -1
        elif x == 'W':
            score += -2
    # not finished
    if white:
        score = -score
    return score


def from_numbering_to_xy(num):
    return ((num * 2 - (1 if (num / 4) % 2 == 0 else 2)) / 8 + 1,
            (num * 2 - (1 if (num / 4) % 2 == 0 else 2)) % 8 + 1)


def gif(z, jump):
    if (z/4) % 2 == 0:
        return int(((z + 1) + jump + 1) / 2) - 1
    else:
        return int(((z + 1) + jump - 1) / 2) - 1


def make_kings(board):
    board = list(board)
    for i in range(28, 32):
        if board[i] == 'b':
            board[i] = 'B'
    for i in range(0, 4):
        if board[i] == 'w':
            board[i] = 'W'
    return str(''.join(board))


def make_move(s_state, move, jump=0):  # makes new state board from the move
    state = list(s_state)
    if not jump:
        a, b = move
        state[a], state[b] = state[b], state[a]
    else:
        a, b = move
        state[a], state[b] = state[b], state[a]
        state[gif(a, b+1)] = '_'
    return str(''.join(state))


def make_move_str(s_state, s_move):
    if s_move == 'WON' or s_move == 'LOST':
        return s_state
    if '-' in s_move:
        move = s_move.split(';')[0]
        print ([int(x) for x in move.split('-')])
        table = make_move(s_state, [(int(x)-1) for x in move.split('-')], jump=False)
        return table
    else:
        table = s_state
        for move in s_move.split(';'):
            if len(move) > 2:
                table = make_move(table, [(int(x)-1) for x in move.split('x')], jump=True)
        return table


def next_state(s_state, white=0):
    if len(s_state) != 32:
        raise ValueError('State is not correct')
    states = []
    kju = deque()
    kju.append((copy.copy(s_state), ""))
    while len(kju) > 0:
        state, prev_moves = kju.popleft()
        moved = 0
        for i in range(32):
            if not white and (state[i] == 'b' or state[i] == 'B'):
                if i + 1 in b_jumps:
                    for jump in b_jumps[i+1]:
                        if state[jump-1] == '_' and \
                                (state[gif(i, jump)] == 'w' or state[gif(i, jump)] == 'W'):
                            kju.append((make_move(state, (i, jump - 1), jump=1),
                                        prev_moves + str(i + 1) + 'x' + str(jump) + ';'))
                            moved = 1
            if not white and (state[i] == 'B'):
                if i + 1 in w_jumps:
                    for jump in w_jumps[i+1]:
                        if state[jump-1] == '_' and \
                                (state[gif(i, jump)] == 'w' or state[gif(i, jump)] == 'W'):
                            kju.append((make_move(state, (i, jump - 1), jump=1),
                                        prev_moves + str(i + 1) + 'x' + str(jump) + ';'))
                            moved = 1
            if white and (state[i] == 'w' or state[i] == 'W'):
                if i + 1 in w_jumps:
                    for jump in w_jumps[i+1]:
                        if state[jump-1] == '_' and \
                                (state[gif(i, jump)] == 'b' or state[gif(i, jump)] == 'B'):
                            kju.append((make_move(state, (i, jump - 1), jump=1),
                                        prev_moves + str(i + 1) + 'x' + str(jump) + ';'))
                            moved = 1
            if white and (state[i] == 'W'):
                if i + 1 in b_jumps:
                    for jump in b_jumps[i+1]:
                        if state[jump-1] == '_' and \
                                (state[gif(i, jump)] == 'b' or state[gif(i, jump)] == 'B'):
                            kju.append((make_move(state, (i, jump - 1), jump=1),
                                        prev_moves + str(i + 1) + 'x' + str(jump) + ';'))
                            moved = 1
        if not moved and len(prev_moves) > 0:
            states.append((make_kings(state), prev_moves))

    if len(states) == 0:
        for i in range(32):
            if not white and (state[i] == 'b' or state[i] == 'B'):
                if i + 1 in b_moves:
                    for jump in b_moves[i + 1]:
                        if state[jump - 1] == '_':
                            states.append((make_kings(make_move(state, (i, jump - 1), jump=0)),
                                           prev_moves + str(i + 1) + '-' + str(jump) + ';'))
            if not white and (state[i] == 'B'):
                if i + 1 in w_moves:
                    for jump in w_moves[i + 1]:
                        if state[jump - 1] == '_':
                            states.append((make_kings(make_move(state, (i, jump - 1), jump=0)),
                                           prev_moves + str(i + 1) + '-' + str(jump) + ';'))
            if white and (state[i] == 'w' or state[i] == 'W'):
                if i + 1 in w_moves:
                    for jump in w_moves[i + 1]:
                        if state[jump - 1] == '_':
                            states.append((make_kings(make_move(state, (i, jump - 1), jump=0)),
                                           prev_moves + str(i + 1) + '-' + str(jump) + ';'))
            if white and (state[i] == 'W'):
                if i + 1 in b_moves:
                    for jump in b_moves[i + 1]:
                        if state[jump - 1] == '_':
                            states.append((make_kings(make_move(state, (i, jump - 1), jump=0)),
                                           prev_moves + str(i + 1) + '-' + str(jump) + ';'))

    for _x in states:
        yield _x


def alpha_beta_search(state, max_depth_s=2, white=False):
    global max_depth
    max_depth = max_depth_s
    v, ret_moves = max_search(state, -INF, INF, white, 0)
    if v == -INF-1:
        ret_moves = (state, "LOST")
    if v == INF+1:
        ret_moves = (state, "WON")
    # print (v, ret_moves)
    return ret_moves


def max_search(state, alpha, beta, white, depth):
    if depth > max_depth:
        return (eval_board(state, white), (state, ''))
    all_moves = next_state(state, white)
    # print "MAX"
    v = -INF-1
    best_moves = ('', '')
    for move in all_moves:
        s, moves_to_s = move
        v_new, ret_moves = min_search(s, alpha, beta, not white, depth+1)
        if v_new > v:
            v = v_new
            best_moves = move
        if v >= beta:
            return (v, best_moves)
        alpha = max(alpha, v)
    return (v, best_moves)


def min_search(state, alpha, beta, white, depth):
    if depth > max_depth:
        return (eval_board(state, white), (state, ''))
    all_moves = next_state(state, white)
    # print "MIN"
    v = INF+1
    best_moves = ('', '')
    for move in all_moves:
        s, moves_to_s = move
        v_new, ret_moves = max_search(s, alpha, beta, not white, depth+1)
        if v_new < v:
            v = v_new
            best_moves = move
        if v <= alpha:
            return (v, best_moves)
        beta = min(beta, v)
    return (v, best_moves)


def convert_board(full_board):
    # TODO checks, rotations, etd.
    t = ""
    for x in full_board:
        if x != '.':
            t += x
    return t


def signal_callback(msg):
    global current_board, last_board, init_board, pub_sig, pub_moves, max_depth
    if msg.data == "AI_INIT_WHITE":
        last_board = current_board = str(reversed(init_board))
    elif msg.data == "AI_INIT_BLACK":
        last_board = current_board = init_board
    elif msg.data == "AI_GO_WHITE":
        print ("usao sa " + current_board)
        v, moves = alpha_beta_search(current_board, max_depth_s=max_depth, white=True)
        pub_moves.publish(moves)
        current_board = make_move_str(current_board, moves)
        print("izas sa " + current_board)
        pub_sig.publish("AI_FINISHED")
    elif msg.data == "AI_GO_BLACK":
        v, moves = alpha_beta_search(current_board, max_depth_s=max_depth, white=False)
        pub_moves.publish(moves)
        current_board = make_move_str(current_board, moves)
        pub_sig.publish("AI_FINISHED")
    elif '=' in msg.data:
        a = msg.data.split('=')
        if a[0] == 'MAX_DEPTH':
            max_depth = int(a[1])


def convert_to_full_board(board):
    z = ""
    flip = False
    for i in range(32):
        if not flip:
            z += '/'
        z += board[i]
        if flip:
            z += '\\'
        if i % 4 == 3:
            z += "\n"
            flip = not flip
    return z


def board_callback(msg):
    global last_board, current_board
    tmp = convert_board(msg.data)
    if tmp and tmp != last_board:
        last_board = current_board
        current_board = tmp


if __name__ == "__main__":
    global pub_moves, pub_sig, current_board
    calculate_all_moves()

    PLAY_SELF = False

    if PLAY_SELF:
        yes = True
        print (gif(13, 21))
        print (convert_to_full_board(current_board))
        for _ in range(77):
            x, y = alpha_beta_search(current_board, white=yes, max_depth_s=4)
            if y == "WON":
                print ("YAY!! ", ("WHITE" if yes else "BLACK") + " WON")
                break
            current_board = x
            print ("THIS IS CURR", current_board, yes)
            yes = not yes
            print ("Move:", _)
            print (convert_to_full_board(current_board))

        eprint("DONE")

    rospy.init_node("checkers_ai")

    pub_moves = rospy.Publisher("/checkers/moves_msg", String, queue_size=50)

    pub_sig = rospy.Publisher("/checkers/ai_sig", String, queue_size=50)

    rospy.Subscriber("/checkers/ai_sig", String, signal_callback, queue_size=50)

    rospy.Subscriber("/checkers/board_msg", String, board_callback, queue_size=50)

    rospy.spin()
