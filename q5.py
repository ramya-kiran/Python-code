#This program implements A star algorithm to find the shortest path(if one available) 
# between two locations with x,y co-ordinates with Chebyshev distance computing the heutistics.


# name : Ramya Rao
# python verison: 2.7.12
# e-mail address: ramrao@umail.iu.edu
import numpy as np
import heapq
import sys
import yaml


# generating the list of nodes that we could reach form each node.
def generating_nodelst(input2, co_ords):
    # to get the length of input2
    m = len(input2)
    dict_nodes = {}
    arr = np.array(input2)
    values = np.unique(arr)
    for i in values:
        dict_nodes[i] = []
    for i in range(m):
        x1, y1 = co_ords[input2[i][0]]
        x2, y2 = co_ords[input2[i][1]]
        if y1 >= y2:
            if input2[i][1] in dict_nodes.keys():
                dict_nodes[input2[i][1]].append(input2[i][0])
            else:
                dict_nodes[input2[i][1]] = input2[i][0]
        elif y2 >= y1:
            if input2[i][0] in dict_nodes.keys():
                dict_nodes[input2[i][0]].append(input2[i][1])
            else:
                dict_nodes[input2[i][0]] = input2[i][1]
    return dict_nodes


# calculating the heuristics distance metrics as a matrix
def distance_h(co_ords):
    m = len(co_ords)
    dist1 = np.ones(shape=(m, m))
    D = 1
    D2 = np.sqrt(2)
    for i in range(m):
        for j in range(m):
            x1, y1 = co_ords[i + 1]
            x2, y2 = co_ords[j + 1]
            dx = abs(x1 - x2)
            dy = abs(y1 - y2)
            func = D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)
            dist1[(i - 1), (j - 1)] = 0
            dist1[(j - 1), (j - 1)] = 0
            dist1[(i - 1), (j - 1)] = func
            dist1[(j - 1), (i - 1)] = func

    return dist1


# Calculating the actual distance between two nodes as a matrix.
def distance_g(highway, co_ords):
    arr = np.array(highway)
    values = np.unique(arr)
    m = len(values)
    dist1 = np.inf * np.ones(shape=(m, m))
    for i in range(len(highway)):
        xval = highway[i][0]
        yval = highway[i][1]
        x1, y1 = co_ords[xval]
        x2, y2 = co_ords[yval]
        func = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
        dist1[(xval-1), (xval-1)] = 0
        dist1[(yval - 1), (yval - 1)] = 0
        dist1[(xval - 1), (yval - 1)] = func
        dist1[(yval - 1), (xval - 1)] = func

    return dist1


# This function gives the optimal distance between two cities
def traversing(cities, nodes, distance_matrix_h, distance_matrix_g, start, end):
    if start not in cities.keys() or end not in cities.keys():
        return "HIGHWAY NOT FOUND BETWEEN CITIES"
    elif cities[start][1] > cities[end][1]:
        return "CANNOT TAKE RIDE DUE TO WIND"
    elif start == end:
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
            if current == end:
                break
            for i in nodes[current]:
                if cities[i][1] >= cities[current][1]:
                    h = distance_matrix_h[i - 1, end - 1]
                    g = visited[current] + distance_matrix_g[current - 1, i-1]
                    if i not in visited or g < visited[i]:
                        visited[i] = g
                        priority = g + h
                        heapq.heappush(fringe, (priority, i))
                        came_from[i] = current
        if end not in visited:
            return "NO PATH EXISTS BETWEEN CITIES"
        else:
            return "travel distance:",visited[end]


# main method which calls all the functions
def main():
    f = open(sys.argv[1], 'r')
    data = yaml.load(f)
    f.close()
    nodes1 = generating_nodelst(data["highways"], data["cities"])
    distance_matrix_h = distance_h(data["cities"])
    distance_matrix_g = distance_g(data["highways"], data["cities"])
    travel_val = traversing(data["cities"], nodes1, distance_matrix_h, distance_matrix_g, data["start"], data["end"])
    print travel_val


if __name__ == "__main__":
    main()

