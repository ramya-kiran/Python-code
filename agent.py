import numpy as np
import heapq
from scipy.spatial.distance import cdist


class Agent:
    def __init__(self):
        self.mat = np.zeros((22, 22))
        self.first_move = True
        self.row = None
        self.col = None
        self.stench_loc = []
        self.wumpus = []
        self.wumpus_found = False
        self.pits = []
        self.killed = False
        self.prev_row = None
        self.prev_col = None
        self.breeze_loc = []
        self.arrow = True
        self.done_with_wumpus = False

    def get_action(self):
        right = (self.row, self.col+1)
        left = (self.row, self.col-1)
        up = (self.row+1, self.col)
        down = (self.row-1, self.col)
        safe_locs = []
        # this is to calculate the immediate safe unexplored location closest to the agent.
        if right not in self.pits and right not in self.wumpus and self.mat[right] == 2:
            safe_locs.append(right)
        if left not in self.pits and left not in self.wumpus and self.mat[left] == 2:
            safe_locs.append(left)
        if down not in self.pits and down not in self.wumpus and self.mat[down] == 2:
            safe_locs.append(down)
        if up not in self.pits and up not in self.wumpus and self.mat[up] == 2:
            safe_locs.append(up)
        # checking if the WUmpus is found and not killed
        if not self.done_with_wumpus and self.wumpus_found and self.arrow:
                act = self.checking((self.wumpus[-1]))
                self.arrow = False
                return "SHOOT_" + act
        # checking if we have any immediate safe neighbors
        else:
            if safe_locs:
                act = self.checking(safe_locs[-1])
                self.prev_row, self.prev_col = safe_locs[-1]
                return "MOVE_" + act
            # if not immediate safe neighbors then check all the unexplored safe locations , find the most nearest
            # location in the matrix and explore that location
            elif not safe_locs:
                possible_next_loc = self.get_nearest()
                if possible_next_loc:
                    find_path = self.traversing((self.row, self.col), possible_next_loc)
                    reconstruct_path = self.path_travel(find_path, (self.row, self.col), possible_next_loc)
                    act = self.checking(reconstruct_path)
                    self.prev_row, self.prev_col = reconstruct_path
                    return "MOVE_" + act
                else:
                    # if nothing found then if not yet used the arrow, use it
                    if not self.killed:
                        if self.arrow:
                            act = self.checking(self.wumpus[-1])
                            return "SHOOT_" + act
                    # if no other option left then QUIT
                    else:
                        return "QUIT"

# this function gives the environmental factors, which will be used to update the matrix for finding out the safe
# unexplored locations.
    def give_senses(self, location, breeze, stench):
        pr_col, pr_row = location
        # First move
        if self.first_move:
            self.mat[pr_row, pr_col] = 1
            self.first_move = False
            self.row = pr_row
            self.col = pr_col
            if not stench and not breeze:
                self.mat[self.row, self.col+1] = 2
                self.mat[self.row, self.col-1] = 2
                self.mat[self.row-1, self.col] = 2
                self.mat[self.row+1, self.col] = 2
            if stench:
                self.wumpus.append((self.row, self.col+1))
                self.wumpus.append((self.row, self.col-1))
                self.wumpus.append((self.row-1, self.col))
                self.wumpus.append((self.row+1, self.col))
            if breeze:
                self.pits.append((self.row, self.col + 1))
                self.pits.append((self.row, self.col - 1))
                self.pits.append((self.row - 1, self.col))
                self.pits.append((self.row + 1, self.col))
        # checking if agent hit a wall
        elif (self.prev_row, self.prev_col) != (pr_row, pr_col):
            self.mat[self.prev_row, self.prev_col] = -1
            if (pr_row, pr_col) in self.wumpus:
                self.wumpus.remove((pr_row, pr_col))
            if (pr_row, pr_col) in self.pits:
                self.pits.remove((pr_row, pr_col))
        # checking if the agent killed the wumpus and if not, noting all the stench locations and possible wumpus
        # locations
        if not self.done_with_wumpus:
            if self.killed:
                row, col = self.wumpus.pop()
                if (row, col) not in self.wumpus:
                    self.mat[row, col] = 2
                    self.done_with_wumpus = True
                    if self.mat[row, col + 1] == 0:
                        self.mat[row, col + 1] = 2
                    if self.mat[row, col - 1] == 0:
                        self.mat[row, col - 1] = 2
                    if self.mat[row + 1, col] == 0:
                        self.mat[row + 1, col] = 2
                    if self.mat[row - 1, col] == 0:
                        self.mat[row - 1, col] = 2
        self.row = pr_row
        self.col = pr_col
        self.mat[self.row, self.col] = 1
        if (self.row, self.col) in self.pits:
            self.pits.remove((self.row, self.col))
        if (self.row, self.col) in self.wumpus:
            self.wumpus.remove((self.row, self.col))
        # if no breeze or stench then the four neighborhood locations are marked safe and unexplored
        if not breeze and not stench:
                if self.mat[self.row, self.col + 1] == 0:
                    self.mat[self.row, self.col + 1] = 2
                if (self.row, self.col+1) in self.pits:
                    self.pits.remove((self.row, self.col+1))
                if (self.row, self.col+1) in self.wumpus:
                    self.wumpus.remove((self.row, self.col + 1))
                if self.mat[self.row, self.col - 1] == 0:
                    self.mat[self.row, self.col - 1] = 2
                if (self.row, self.col-1) in self.pits:
                    self.pits.remove((self.row, self.col-1))
                if (self.row, self.col-1) in self.wumpus:
                    self.wumpus.remove((self.row, self.col - 1))
                if self.mat[self.row + 1, self.col] == 0:
                    self.mat[self.row + 1, self.col] = 2
                if (self.row+1, self.col) in self.pits:
                    self.pits.remove((self.row+1, self.col))
                if (self.row+1, self.col) in self.wumpus:
                    self.wumpus.remove((self.row+1, self.col))
                if self.mat[self.row - 1, self.col] == 0:
                    self.mat[self.row - 1, self.col] = 2
                if (self.row-1, self.col) in self.pits:
                    self.pits.remove((self.row-1, self.col))
                if (self.row-1, self.col) in self.wumpus:
                    self.wumpus.remove((self.row-1, self.col))
        # if there is breeze then the corresponding 4 neighbor locations are included in the pits list.
        if breeze:
            if self.mat[self.row, self.col + 1] == 0:
                if (self.row, self.col + 1) not in self.pits:
                    self.pits.append((self.row, self.col + 1))
            if self.mat[self.row + 1, self.col] == 0:
                if (self.row + 1, self.col) not in self.pits:
                    self.pits.append((self.row + 1, self.col))
            if self.mat[self.row, self.col - 1] == 0:
                if (self.row, self.col - 1) not in self.pits:
                    self.pits.append((self.row, self.col - 1))
            if self.mat[self.row - 1, self.col] == 0:
                if (self.row - 1, self.col) not in self.pits:
                    self.pits.append((self.row - 1, self.col))
        # if there is stench and the wumpus is not killed then mark all the corresponding four neighborhood positions
        # as unsafe.
        if stench:
            if not self.killed:
                if (self.row, self.col) not in self.stench_loc:
                    self.stench_loc.append((self.row, self.col))
                if len(self.stench_loc) == 1:
                    right = (self.row, self.col + 1)
                    if right:
                        if self.mat[right] != -1 and self.mat[right] == 0:
                            self.wumpus.append(right)
                    left = (self.row, self.col - 1)
                    if left:
                        if self.mat[left] != -1 and self.mat[left] == 0:
                            self.wumpus.append(left)
                    down = (self.row - 1, self.col)
                    if down:
                        if self.mat[down] != -1 and self.mat[down] == 0:
                            self.wumpus.append(down)
                    up = (self.row + 1, self.col)
                    if up:
                        if self.mat[up] != -1 and self.mat[up] == 0:
                            self.wumpus.append(up)
                # with two stench positions, the position of the wumpus is narrowed down to two locations
                elif len(self.stench_loc) == 2:
                    first_set = self.neighbors(self.stench_loc[0])
                    second_set = self.neighbors(self.stench_loc[1])
                    collect_com = list(set(first_set).intersection(second_set))
                    sub_com = list(set(self.wumpus) - set(collect_com))
                    for i in sub_com:
                        if i not in self.pits:
                            if self.mat[i] == 0:
                                self.mat[i] = 2
                    self.wumpus = []
                    for p in collect_com:
                        if self.mat[p] == 0:
                            self.wumpus.append(p)
                    if len(self.wumpus) == 1:
                        self.wumpus_found = True
                # if there are three stench positions then the wumpus location is definitely found.
                elif len(self.stench_loc) == 3:
                    first_set = self.neighbors(self.stench_loc[0])
                    second_set = self.neighbors(self.stench_loc[1])
                    third_set = self.neighbors(self.stench_loc[2])
                    collect_com = list(set(first_set).intersection(set(second_set).intersection(third_set)))
                    sub_com = list(set(self.wumpus) - set(collect_com))
                    for i in sub_com:
                        if i not in self.pits:
                            if self.mat[i] == 0:
                                self.mat[i] = 2
                    self.wumpus = []
                    for p in collect_com:
                        if self.mat[p] == 0:
                            self.wumpus.append(p)
                    if len(self.wumpus) == 1:
                        self.wumpus_found = True
            # if the Wumpus is already killed then the corresponding 4 neighbors are safe if not present in the pits.
            else:
                if self.mat[self.row, self.col + 1] == 0:
                    if (self.row, self.col + 1) not in self.pits:
                        self.mat[self.row, self.col + 1] = 2
                if self.mat[self.row, self.col - 1] == 0:
                    if (self.row, self.col - 1) not in self.pits:
                        self.mat[self.row, self.col - 1] = 2
                if self.mat[self.row + 1, self.col] == 0:
                    if (self.row + 1, self.col) not in self.pits:
                        self.mat[self.row + 1, self.col] = 2
                if self.mat[self.row - 1, self.col] == 0:
                    if (self.row - 1, self.col) not in self.pits:
                        self.mat[self.row - 1, self.col] = 2

    # this function is called when the Wumpus is killed
    def killed_wumpus(self):
        self.killed = True

# this function is go give the direction to move from the location the agent is in.
    def checking(self, some):
        r, c = some
        if r < self.row:
            return "DOWN"
        elif r > self.row:
            return "UP"
        elif c < self.col:
            return "LEFT"
        elif c > self.col:
            return "RIGHT"

# generating 4 - neighbors for calculating the Wumpus location
    def neighbors(self, loc):
        hor, ver = loc
        explore = []
        right = hor, ver + 1
        if self.mat[right] != -1:
            explore.append(right)
        left = hor, ver - 1
        if self.mat[left] != -1:
            explore.append(left)
        down = hor - 1, ver
        if self.mat[down] != -1:
            explore.append(down)
        up = hor + 1, ver
        if self.mat[up] != -1:
            explore.append(up)
        return explore

# this function is used to return the safe-neighbours for the a-star algorithm
    def neighbors_astar(self, loc):
        hor, ver = loc
        explore = []
        right = hor, ver + 1
        if right not in self.pits and right not in self.wumpus and self.mat[right] != -1:
            if self.mat[right] != 0:
                explore.append(right)
        left = hor, ver - 1
        if left not in self.pits and left not in self.wumpus and self.mat[left] != -1:
            if self.mat[left] != 0:
                explore.append(left)
        down = hor - 1, ver
        if down not in self.pits and down not in self.wumpus and self.mat[down] != -1:
            if self.mat[down] != 0:
                explore.append(down)
        up = hor + 1, ver
        if up not in self.pits and left not in self.wumpus and self.mat[up] != -1:
            if self.mat[up] != 0:
                explore.append(up)
        return explore

# implementing a star algorithm with manhattan distance for heuristics for determining the shortest path from start to
    # the destination
    def traversing(self, start, end):
        if np.array_equal(start, end):
            return 0
        else:
            fringe = []
            heapq.heappush(fringe, (0, start))
            visited = {}
            visited[start] = 0
            came_from = {}
            came_from[start] = None
            while not (len(fringe) == 0):
                current = heapq.heappop(fringe)[1]
                if np.array_equal(current, end):
                    break
                places = self.neighbors_astar(current)
                if places:
                    for i in places:
                        h = abs(i[0] - end[0]) + abs(i[1] - end[1])
                        g = visited[current] + 1
                        if i not in visited or g < visited[i]:
                            visited[i] = g
                            priority = g + h
                            heapq.heappush(fringe, (priority, i))
                            came_from[i] = current
            if tuple(end) not in visited:
                return -1
            else:
                return came_from

    # from the came_from dictionary from the a star algorithm,, reconstructing the path from start to destination.
    def path_travel(self, came_from, start, end):
        curr = end
        path = [curr]
        while curr != start:
            curr = came_from[tuple(curr)]
            path.append(curr)
        path.reverse()
        path = path[1]
        return path

# the function calculates all the safe unexplored locations
    def get_nearest(self):
        actual = []
        co_ords = np.where(self.mat == 2)
        if co_ords[0].size != 0:
            for i in range(len(co_ords[0])):
                actual.append((co_ords[0][i], co_ords[1][i]))
            input1 = [self.row, self.col]
            another = []
            for i in actual:
                if i not in self.pits and i not in self.wumpus:
                    another.append(i)
            nearest_loc = actual[cdist(np.array([input1]), np.array(actual)).argmin()]
            return nearest_loc
        else:
            return None

