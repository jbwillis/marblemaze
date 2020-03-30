import numpy as np
import matplotlib.pyplot as plt
import time

#plots graph with nodes and freespace edges
def plot_board(size, edges, goal):
    plt.figure(1)
    w = size[0]
    h = size[1]

    x_row = [i for i in range(w)]
    x = []
    y = []
    for i in range(h):
        x += x_row
        y += [-i for j in range(w)]
    plt.scatter(x,y)

    for i in range(w*h):
        xval = i%w
        yval = -np.floor(i/w)
        for j in range(4):
            if edges[i,j]:
                if j == 0: #up
                    plt.plot([xval,xval],[yval,yval+1])
                elif j == 1: #down
                    plt.plot([xval,xval],[yval,yval-1])
                elif j == 2: #left
                    plt.plot([xval,xval-1],[yval,yval])
                elif j == 3: #right
                    plt.plot([xval,xval+1],[yval,yval])

    plt.plot(goal%w, -np.floor(goal/w), 'r*')

def plot_solution(size, policy, goal):
    plt.figure(2)
    w = size[0]
    h = size[1]

    x_row = [i for i in range(w)]
    x = []
    y = []
    for i in range(h):
        x += x_row
        y += [-i for j in range(w)]
    plt.scatter(x,y)

    policy = np.array([policy]).reshape((h,w)).astype(int)
    for i in range(w*h):
        x = int(i%w)
        y = int(-np.floor(i/w))
        direct = policy[-y,x]
        if direct == 0: #up
            dx = 0
            dy = 1
        elif direct == 1: #down
            dx = 0
            dy = -1
        elif direct == 2: #left
            dx = -1
            dy = 0
        elif direct == 3: #right
            dx = 1
            dy = 0
        if direct != -1:
            plt.arrow(x, y, dx, dy, width=0.05, length_includes_head=True)

    plt.plot(goal%w, -np.floor(goal/h), 'r*')

def out_of_bounds(size, node, edge):
    w = size[0]
    h = size[1]
    if node < w and edge == 0:
        return True
    if node > w*(h-1) - 1 and edge == 1:
        return True
    if not node%w and edge == 2:
        return True
    if not (node+1)%w and edge == 3:
        return True
    return False


def make_random_maze(size, num_walls):
    w = size[0]
    h = size[1]
    #each row is [up, down, left, right]
    #1=opening, 0 = wall
    edges = np.ones((w*h,4))
    #No edges out of bounds
    for i in range(w*h):
        if i < w: #top nodes
            edges[i,0] = 0
        if i > w*(h-1) - 1: #bottom nodes
            edges[i,1] = 0
        if not i % w: #left nodes
            edges[i,2] = 0
        if not (i+1) % w: #right nodes
            edges[i,3] = 0

    #create random walls
    count = 0
    while count < num_walls:
        node = np.random.randint(0,w*h)
        edge = np.random.randint(0,4)
        if not out_of_bounds((w,h), node, edge) and edges[node, edge]:
            edges[node, edge] = 0
            if edge == 0:
                edges[node-w, 1] = 0
            elif edge == 1:
                edges[node+w, 0] = 0
            elif edge == 2:
                edges[node-1, 3] = 0
            elif edge == 3:
                edges[node+1, 2] = 0
            count += 1

    return edges

#breadth first search. Takes board size, graph edges, and goal node
#returns a list representing which direction to go for every node
#0 = up, 1 = down, 2 = left, 3 = right
def bfs(size, edges, goal):
    w = size[0]
    h = size[1]

    dist = np.inf * np.ones(w*h)
    policy = -1 * np.ones(w*h)

    dist[goal] = 0
    q = [goal]
    while len(q):
        u = q.pop(0)
        adj = edges[u,:]
        for i,j in enumerate(adj):
            if j:
                if i == 0:
                    v = u - w
                    direct = 1
                elif i == 1:
                    v = u + w
                    direct = 0
                elif i == 2:
                    v = u - 1
                    direct = 3
                elif i == 3:
                    v = u + 1
                    direct = 2

                if dist[v] == np.inf:
                    q.append(v)
                    dist[v] = dist[u] + 1
                    policy[v] = direct

    return policy, dist




if __name__=="__main__":
    board_w = 12
    board_h = 12
    num_walls = 100

    goal = 143

    #nodes run from left to right, top to bottom
    nodes = [i for i in range(board_w*board_h)]
    edges = make_random_maze((board_w,board_h), num_walls)

    start = time.time()
    policy, dist = bfs((board_w,board_h), edges, goal)
    print(time.time()-start)
    print(np.array([policy]).reshape((board_h,board_w)))

    plot_board((board_w,board_h), edges, goal)
    plot_solution((board_w,board_h), policy, goal)
    plt.show()
