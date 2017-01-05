import board


# this function just generates a search tree of depth given and returns the number of leaf nodes.
def perft(b, depth):
    b.count = 0
    b.count = another(b, depth)
    return b.count


# This function runs recursively until the depth is reached and return the number of leaf nodes.
def another(b, depth):
    if not b.stack or depth == 0 or b.last_move_won():
        b.count += 1
        return b.count
    else:
        moves1 = b.generate_moves()
        for move in moves1:
            b.make_move(move)
            another(b, depth - 1)
            b.unmake_last_move()
        return b.count


# This function runs alpha beta pruning to return a string with information about the future
#  of the player within the depth.
def find_win(board, depth):
    utility = 100
    val, move = alpha_beta(board, -utility - 1, utility + 1, depth)
    if val == utility:
        return "WIN BY PLAYING " + str(move)
    elif val == -utility:
        return "ALL MOVES LOSE"
    else:
        return "NO FORCED WIN IN " + str(depth) + " MOVES"


# Alpha beta pruning function
def alpha_beta(board, alpha, beta, depth):
    if board.last_move_won():
        return -100, None
    elif depth == 0 or not board.stack:
        return 0, None
    else:
        moves = board.generate_moves()
        best_move = None
        for m in moves:
            board.make_move(m)
            v, _ = alpha_beta(board, -beta, -alpha, depth - 1)
            v = -v
            board.unmake_last_move()
            if v >= beta:
                return beta, m
            if v > alpha:
                alpha = v
                best_move = m
        return alpha, best_move

