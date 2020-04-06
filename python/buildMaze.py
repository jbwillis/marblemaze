def buildMaze(maze):
    mazeLen = 12
    mazeWallThick = 17
    firstWallThick = 7
    mazeGapThick = 16
    mazeGapAndWallThick = mazeWallThick + mazeGapThick
    checkSize = (mazeWallThick * mazeGapThick) * 0.3
    nodalMaze = np.zeros((mazeLen * mazeLen, 4))

    mazeX = firstWallThick
    for k in range(mazeLen):
        mazeY = firstWallThick
        for j in range(mazeLen):
            if k == 0:
                up = 0;
            else:
                up = int(np.count_nonzero(maze[mazeX-mazeWallThick:mazeX,mazeY:mazeY+mazeGapThick]) < checkSize)
            if k == mazeLen - 1:
                down = 0
            else:
                down = int(np.count_nonzero(maze[mazeX + mazeGapThick:mazeX + mazeGapAndWallThick, mazeY:mazeY + mazeGapThick]) < checkSize)
            if j == 0:
                left = 0
            else:
                left = int(np.count_nonzero(maze[mazeX:mazeX+mazeGapThick,mazeY-mazeWallThick:mazeY]) < checkSize)
            if j == mazeLen - 1:
                right = 0
            else:
                right = int(np.count_nonzero(maze[mazeX:mazeX+mazeGapThick,mazeY+mazeGapThick:mazeY+mazeGapAndWallThick]) < checkSize)
            nodalMaze[k*mazeLen + j, :] = np.array([up,down,left,right]).reshape(1,4)
            mazeY = mazeY + mazeGapAndWallThick
        mazeX = mazeX + mazeGapAndWallThick

    return nodalMaze