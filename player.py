import random
import board
import search

# find win, find the best possible move from any board state, if any present.
def find_win(board, depth):
    utility = 100
    val, move = search.alpha_beta(board, -utility - 1, utility + 1, depth)
    return move


class Player:
    # initialising the board state of the player.
    def __init__(self):
        self.b = board.Board()
        self.value = 1
        self.opponent = -1

    # Just generating a random name
    def name(self):
        return 'Bliss'

    # making a move and updating the board state of the player
    def make_move(self, move):
        self.b.make_move(move)

    # get legal moves from the find win function.
    def get_move(self):
        move = find_win(self.b, 8)
        return move
