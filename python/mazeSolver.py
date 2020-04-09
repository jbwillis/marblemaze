import cv2 as cv
import numpy as np

class mazeSolver:
    def __init__(self, solve_every_time, show_stuff):
        self.solve_every_time = solve_every_time
        self.solved = False
        self.graph = None
        self.policy = None
        self.board_size = (12, 12)

        self.x_grid = np.linspace(20, 380, 12, dtype=int)
        self.y_grid = self.x_grid

        self.show_stuff = show_stuff

        self.dst_corners = np.float32([[0,0],[0,400],[400,0],[400,400]])
        self.perspective_size = (400,400)

        self.mazeBGRLow = np.array([0,0,0])
        self.mazeBGRHigh = np.array([120,120,120])

        # self.marbleHSVLow = np.array([30,60,100])
        # self.marbleHSVHigh = np.array([50,150,255])
        # self.marbleHSVLow = np.array([30,50,100])
        # self.marbleHSVHigh = np.array([50,100,255])

        # blue ball
        self.marbleHSVLow = np.array([105,100,80])
        self.marbleHSVHigh = np.array([120,255,255])


    def get_max_contour(self, binary):
        contours, hierarchy = cv.findContours(binary, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        if not len(contours):
            return None

        areas = [cv.contourArea(c) for c in contours]
        max_index = np.argmax(areas)
        return contours[max_index]


    def get_corners(self, frame):
        frame = cv.medianBlur(frame, 5)
        ret,walls_b = cv.threshold(frame[:,:,0],90,255,cv.THRESH_BINARY_INV)
        ret,walls_g = cv.threshold(frame[:,:,1],80,255,cv.THRESH_BINARY_INV)
        ret,walls_r = cv.threshold(frame[:,:,2],80,255,cv.THRESH_BINARY_INV)
        walls = cv.bitwise_or(cv.bitwise_or(walls_g, walls_b), walls_r)

        erosion_size = 1
        element = cv.getStructuringElement(cv.MORPH_ELLIPSE, (2*erosion_size + 1, 2*erosion_size+1), (erosion_size, erosion_size))
        walls = cv.erode(walls, element)
        walls = cv.dilate(walls, element)

        erosion_size = 2
        element = cv.getStructuringElement(cv.MORPH_ELLIPSE, (2*erosion_size + 1, 2*erosion_size+1), (erosion_size, erosion_size))
        walls = cv.dilate(walls, element)
        walls = cv.erode(walls, element)

        wall_cnt = self.get_max_contour(walls)

        epsilon = 0.1*cv.arcLength(wall_cnt, True)
        approx = cv.approxPolyDP(wall_cnt, epsilon, True).reshape(4,2)
        approx = approx[np.argsort(approx[:, 0])]
        left_y = np.argmin([approx[0,1],approx[1,1]])
        right_y = np.argmin([approx[2,1],approx[3,1]])
        if left_y:
            approx[[0, 1]] = approx[[1, 0]]
        if right_y:
            approx[[2, 3]] = approx[[3, 2]]

        return approx


    def perspectiveWarp(self, frame):
        try:
            corners = self.get_corners(frame)
        except:
            # image wasn't complete
            print("Bad frame!")
            return None

        H,_ = cv.findHomography(corners, self.dst_corners)
        warped = cv.warpPerspective(frame, H, self.perspective_size)
        return warped


    def filterMazeNoise(self, img):
        kernel7 = np.ones((7,7),np.uint8)
        kernel3 = np.ones((1,1),np.uint8)

        img = cv.morphologyEx(img, cv.MORPH_CLOSE, kernel7, iterations=1)
        img = cv.dilate(img,kernel7,iterations=3)
        img = cv.erode(img,kernel7,iterations=2)
        # img = cv.dilate(img,kernel3,iterations=2)

        return img


    def filterMarbleNoise(self, img):
        kernel = np.ones((5,5),np.uint8)

        img = cv.erode(img,kernel, iterations=1)
        img = cv.dilate(img,kernel,iterations=2)
        img = cv.erode(img,kernel, iterations=2)
        img = cv.dilate(img,kernel, iterations=2)

        return img


    def segmentImg(self, frame):
        frame = cv.medianBlur(frame, 5)

        # find marble
        frameHSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        marbleframe = cv.inRange(frameHSV, self.marbleHSVLow, self.marbleHSVHigh)
        cv.imshow('marble_1', marbleframe)
        marbleframe = self.filterMarbleNoise(marbleframe)
        cv.imshow('marble_2', marbleframe)

        marbleframe = cv.Canny(marbleframe,100,255)
        cv.imshow('marble_conts', marbleframe)

        marble = self.get_max_contour(marbleframe)
        marblerect = None
        if marble is not None:
            center, rad = cv.minEnclosingCircle(marble)
            if rad < 10.0:
                print("Where's the marble?!!")
                center = None # didn't actually find the marble
            else:
                marblerect = cv.boundingRect(marble)
        else:
            print("Where's the marble?!!")
            center = None


        # find maze walls
        maze = cv.inRange(frame, self.mazeBGRLow, self.mazeBGRHigh)
        cv.imshow("maze marble", maze)

        if marblerect is not None:
            x, y, w, h = marblerect
            # remove marble from maze
            cv.rectangle(maze, (x-2,y-2), (x+w+2, y+h+2), 0, -1) # black, filled rectangle


        maze = self.filterMazeNoise(maze)
        cv.imshow("maze no marble", maze)

        return maze, center


    # def buildGraph(self, maze):
    #     mazeLen = 12
    #     mazeWallThick = 17
    #     firstWallThick = 7
    #     mazeGapThick = 16
    #     mazeGapAndWallThick = mazeWallThick + mazeGapThick
    #     # checkSize = (mazeWallThick * mazeGapThick) * 0.3
    #     checkSize = 10
    #     nodalMaze = np.zeros((mazeLen * mazeLen, 4))
    #
    #     mazeX = firstWallThick
    #     for k in range(mazeLen):
    #         mazeY = firstWallThick
    #         for j in range(mazeLen):
    #             if k == 0:
    #                 up = 0;
    #             else:
    #                 up = int(np.count_nonzero(maze[mazeX-mazeWallThick:mazeX,mazeY:mazeY+mazeGapThick]) < checkSize)
    #             if k == mazeLen - 1:
    #                 down = 0
    #             else:
    #                 down = int(np.count_nonzero(maze[mazeX + mazeGapThick:mazeX + mazeGapAndWallThick, mazeY:mazeY + mazeGapThick]) < checkSize)
    #             if j == 0:
    #                 left = 0
    #             else:
    #                 left = int(np.count_nonzero(maze[mazeX:mazeX+mazeGapThick,mazeY-mazeWallThick:mazeY]) < checkSize)
    #             if j == mazeLen - 1:
    #                 right = 0
    #             else:
    #                 right = int(np.count_nonzero(maze[mazeX:mazeX+mazeGapThick,mazeY+mazeGapThick:mazeY+mazeGapAndWallThick]) < checkSize)
    #             nodalMaze[k*mazeLen + j, :] = np.array([up,down,left,right]).reshape(1,4)
    #             mazeY = mazeY + mazeGapAndWallThick
    #         mazeX = mazeX + mazeGapAndWallThick
    #
    #     return nodalMaze

    def buildGraph(self, maze):
        mazeLen = 12
        checkSize = 1
        nodalMaze = np.zeros((mazeLen * mazeLen, 4))

        for i in range(mazeLen): #down
            for j in range(mazeLen): #right
                if i == 0:
                    up = 0;
                else:
                    up = int(np.count_nonzero(maze[self.y_grid.item(i-1):self.y_grid.item(i), self.x_grid.item(j)]) < checkSize)
                if i == mazeLen - 1:
                    down = 0
                else:
                    down = int(np.count_nonzero(maze[self.y_grid.item(i):self.y_grid.item(i+1), self.x_grid.item(j)]) < checkSize)
                if j == 0:
                    left = 0
                else:
                    left = int(np.count_nonzero(maze[self.y_grid.item(i), self.x_grid.item(j-1):self.x_grid.item(j)]) < checkSize)
                if j == mazeLen - 1:
                    right = 0
                else:
                    right = int(np.count_nonzero(maze[self.y_grid.item(i), self.x_grid.item(j):self.x_grid.item(j+1)]) < checkSize)
                nodalMaze[i*mazeLen + j, :] = np.array([up,down,left,right]).reshape(1,4)

        return nodalMaze



    def bfs(self, size, edges, goal):
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


    # Takes the unwarped image and solves the maze. Returns marble position and
    # next waypoint position. Returns None (but still solves maze) if marble is
    # not found. Returns None if the current frame is bad. Solves maze every
    # time if self.solve_every_time is True. Otherwise, solves at the first call
    # or every time the goal node is changed.
    def solveMaze(self, frame, goal):
        self.warped = self.perspectiveWarp(frame)
        if self.warped is None:
            return None, None

        self.maze_walls, self.marble_pos = self.segmentImg(self.warped)

        if self.solve_every_time or not self.solved or goal != self.goal:
            self.graph = self.buildGraph(self.maze_walls)
            self.policy,_ = self.bfs(self.board_size, self.graph, goal)
            self.goal = goal
            self.solved = True

        if self.marble_pos is None:
            return None, None

        marble_x_closest = np.argmin(abs(self.marble_pos[0] - self.x_grid))
        marble_y_closest = np.argmin(abs(self.marble_pos[1] - self.y_grid))
        current_grid_x = self.x_grid.item(marble_x_closest)
        current_grid_y = self.y_grid.item(marble_y_closest)

        goal_pos = (int(self.goal%12.0), int(np.floor(self.goal/12.0)))

        policy_grid = np.array([self.policy]).reshape(self.board_size).astype(int)
        direction = policy_grid[marble_y_closest, marble_x_closest]
        if direction == 0:
            waypoint = (current_grid_x, self.y_grid.item(marble_y_closest-1))
        elif direction == 1:
            waypoint = (current_grid_x, self.y_grid.item(marble_y_closest+1))
        elif direction == 2:
            waypoint = (self.x_grid.item(marble_x_closest-1), current_grid_y)
        elif direction == 3:
            waypoint = (self.x_grid.item(marble_x_closest+1), current_grid_y)
        elif direction == -1 and (marble_x_closest, marble_y_closest) == goal_pos:
            print("Goal!!!!")
            waypoint = None
        else:
            print("This maze is unsolvable!")
            waypoint = None


        if self.show_stuff:
            self.showCoolStuff()

        return self.marble_pos, waypoint


    def showCoolStuff(self):
        if self.maze_walls is not None:
            cv.imshow('maze walls', self.maze_walls)

        if self.marble_pos is not None and self.warped is not None:
            warp_draw = self.warped.copy()
            marble_center = (int(self.marble_pos[0]), int(self.marble_pos[1]))
            cv.circle(warp_draw, marble_center, 15, (255,0,0), 2)

            policy_grid = np.array([self.policy]).reshape(self.board_size).astype(int)
            for i in range(12): #down
                for j in range(12): #right
                    cv.circle(warp_draw, (self.x_grid.item(i),self.y_grid.item(j)), 3, (0,0,0), -1)
                    direct = policy_grid[i,j]
                    if direct == 0:
                        cv.arrowedLine(warp_draw, (self.x_grid.item(j),self.y_grid.item(i)), (self.x_grid.item(j),self.y_grid.item(i-1)), (0,0,255), tipLength=0.3)
                    elif direct == 1:
                        cv.arrowedLine(warp_draw, (self.x_grid.item(j),self.y_grid.item(i)), (self.x_grid.item(j),self.y_grid.item(i+1)), (0,0,255), tipLength=0.3)
                    elif direct == 2:
                        cv.arrowedLine(warp_draw, (self.x_grid.item(j),self.y_grid.item(i)), (self.x_grid.item(j-1),self.y_grid.item(i)), (0,0,255), tipLength=0.3)
                    elif direct == 3:
                        cv.arrowedLine(warp_draw, (self.x_grid.item(j),self.y_grid.item(i)), (self.x_grid.item(j+1),self.y_grid.item(i)), (0,0,255), tipLength=0.3)

            goal_pos = (self.x_grid.item(int(self.goal%12.0)), self.y_grid.item(int(np.floor(self.goal/12.0))))
            cv.circle(warp_draw, goal_pos, 5, (0,255,0), -1)

            cv.imshow('marble', warp_draw)
