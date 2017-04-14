import numpy as np
import cv2
import math

class FMM(object):
    """description of class"""

    map = np.array
    time = np.array
    speed = np.array

    cSet = np.array
    oSet = np.array
    gScore = np.array
    fScore = np.array

    oVec = []

    cell2Meter = 0.5

    crash_radius_m = 0.5
    crash_radius_c = crash_radius_m / cell2Meter

    inflation_radius_m = 4.0
    inflation_radius_c = inflation_radius_m / cell2Meter

    start = [0,0]
    goal = [90,90]

    def __init__(self, map, max_speed, cell_to_meter, start, goal ):

        self.start = start
        self.goal = goal

        self.max_speed = max_speed

        self.map = map
        self.time = np.ones( np.shape(self.map ) )*-1
        self.speed = np.ones( np.shape(self.map ) )*self.max_speed
        
        self.cSet = np.zeros( np.shape( self.map ) )
        self.oSet = np.zeros( np.shape( self.map ) )
        self.fScore = np.zeros( np.shape( self.map ) )

        oVec = []

        for i in range(0, np.size( self.map, 0) ):
            for j in range(0, np.size( self.map, 1) ):
                if self.map[i][j] > 0:
                    self.speed[i][j] = 0.0

    def findPath(self, start, goal):
         
        path = []
        dist = math.sqrt( pow(start[0] - goal[0], 2) + pow(start[1] - goal[1], 2) )
        prior_dist = dist + 1
        flag = True

        mod_scale = 1.0
        
        c = [goal[0]+0.0, goal[1]+0.0]
        while( flag or dist < prior_dist):
            if( dist < 5* self.max_speed ):
                flag = False

            p = [c[0]+0.0, c[1]+0.0]
            path.append( p )
            g = self.getGradient( c )

            mod = mod_scale* math.sqrt( pow(g[0],2) + pow(g[1],2) )
            alpha = math.atan2( g[1], g[0] )

            c[0] -= mod*math.cos( alpha )
            c[1] -= mod*math.sin( alpha )

            prior_dist = dist
            dist = math.sqrt( pow(start[0] - c[0], 2) + pow(start[1] - c[1], 2) )

        return path

    def getGradient( self, p ):
        
        pr = [round(p[0]), round(p[1])]
        
        nx = [pr[0]-1, pr[0], pr[0]+1]
        ny = [pr[1]-1, pr[1], pr[1]+1]

        t_nbrs = np.zeros( (3,3) )

        for x in range(0,3):
            for y in range(0,3):
                if self.time[nx[x]][ny[y]] == -1:
                    self.wavePropagation( self.start, [nx[x], ny[y]] )
                
                t_nbrs[x][y] = self.time[nx[x]][ny[y]]
                          
        dx = np.zeros( (3,3) )
        dy = np.zeros( (3,3) )

        dx[0][0] = -1
        dx[0][1] = -2
        dx[0][2] = -1
        dx[2][0] = 1
        dx[2][1] = 2
        dx[2][2] = 1

        dy[0][0] = -1
        dy[1][0] = -2
        dy[2][0] = -1
        dy[0][2] = 1
        dy[1][2] = 2
        dy[2][2] = 1

        g = [0.0, 0.0]
        for x in range(0,3):
            for y in range(0,3):
                g[0] += dx[x][y] * t_nbrs[x][y]
                g[1] += dy[x][y] * t_nbrs[x][y]

        g[0] /= 4
        g[1] /= 4
        
        return g

    def wavePropagation(self, start, goal):
        if math.fabs(start[0] - goal[0]) + math.fabs(start[1] - goal[1]) < self.max_speed:
            return

        if len( self.oVec ) == 0:
            self.oVec.append( start )
            self.oSet[start[0]][start[1]] = 1
            self.fScore[start[0]][start[1]] = math.sqrt( pow(start[0] - goal[0],2) + pow(start[1] - goal[1],2) )
        else:
            for o in self.oVec:
                self.fScore[o[0]][o[1]] = math.sqrt( pow(o[0] - goal[0],2) + pow(o[1] - goal[1],2) )


        n_diff = [[-1,0], [1,0], [0,-1], [0,1], [1,1], [-1,-1], [1,-1], [-1,1]]
    
        while len( self.oVec ) > 0:
            # this finds node with lowest fScore and makes current
            min = float("inf")
            mindex = -1

            for o in self.oVec:
                if self.fScore[o[0]][o[1]] < min:
                    min = self.fScore[o[0]][o[1]]
                    cLoc = o
                    mindex = 1

            if mindex == -1:
                return
        
            self.oVec.remove(cLoc)
            self.oSet[cLoc[0]][cLoc[1]] = 0
            self.cSet[cLoc[0]][cLoc[1]] = 1
        
            if cLoc == goal: # if the current node equals goal, return wave
                return

            for n in n_diff:
                nbr = [cLoc[0] + n[0], cLoc[1] + n[1]]
                if nbr[0] >= 0 and nbr[0] < np.size( self.map, 0) and nbr[1] >= 0 and nbr[1] < np.size(self.map,1): # is nbr on mat?
                    if(self.cSet[nbr[0]][nbr[1]] == 1): # has it already been eval? in cSet
                        continue
                
                    dist = math.sqrt( pow( cLoc[0]-nbr[0], 2) + pow(cLoc[1]-nbr[1], 2))
                    avg_vel = max(0.0000001,  (self.speed[cLoc[0]][cLoc[1]] + self.speed[nbr[0]][nbr[1]])/2)
                    ngScore = self.time[cLoc[0]][cLoc[1]] + dist / avg_vel
                
                    if self.oSet[nbr[0]][nbr[1]] == 0:
                        self.oSet[nbr[0]][nbr[1]] = 1  # add nbr to open set
                        self.oVec.append(nbr)
                    elif ngScore >= self.time[nbr[0]][nbr[1]]: # is temp gscore worse than stored g score of nbr
                        continue
                
                    self.time[nbr[0]][nbr[1]] = ngScore
                    if self.speed[nbr[0]][nbr[1]] > 0.0:
                        dist = math.sqrt( pow( nbr[0]-goal[0], 2) + pow(nbr[1]-goal[1], 2))
                        self.fScore[nbr[0]][nbr[1]] = self.time[nbr[0]][nbr[1]] + dist / self.max_speed
                    else:
                        self.fScore[nbr[0]][nbr[1]] = float("inf")

        return

    def displayMap( self ):

        display = np.ones( (np.size( self.map, 0),np.size( self.map, 1),3), np.uint8)*255 
        for i in range(0, np.size( self.map, 0) ):
            for j in range(0, np.size( self.map, 1) ):
                if self.map[i][j] > 0:
                    display[i][j] = 0

        cv2.circle( display, (self.start[0], self.start[1]), 4, (127, 127,127), -1)
        cv2.circle( display, (self.goal[0], self.goal[1]), 4, (127,127,127), -1)
        cv2.namedWindow("fm2 Map", cv2.WINDOW_NORMAL)
        cv2.imshow("fm2 Map", display)
        cv2.waitKey(10)
    
    def displaySpeedMap( self ):

        display = np.zeros( (np.size(self.map, 0), np.size(self.map, 1) ), np.uint8)
        for i in range(0, np.size( self.map, 0) ):
            for j in range(0, np.size( self.map, 1) ):
                val = self.max_speed - self.speed[i][j] / self.max_speed
                shade = 255 - round( val*255 )
                display[i][j] = shade

        cv2.circle( display, (self.start[0], self.start[1]), 4, (127), -1)
        cv2.circle( display, (self.goal[0], self.goal[1]), 4, (127), -1)
        
        cv2.namedWindow("fm2 speedMap", cv2.WINDOW_NORMAL)
        cv2.imshow("fm2 speedMap", display)
        cv2.waitKey(10)

    def displayTime( self ):
        max = np.max( np.max( self.time ) )
        display = np.zeros( (np.size(self.map, 0), np.size(self.map, 1) ), np.uint8)
        for i in range(0, np.size( self.map, 0) ):
            for j in range(0, np.size( self.map, 1) ):
                val = (max - self.time[i][j]) / (max)
                shade = 255 - round( val*255 )
                display[i][j] = shade
    
        cv2.circle( display, (self.start[0], self.start[1]), 4, (127), -1)
        cv2.circle( display, (self.goal[0], self.goal[1]), 4, (127), -1)
    
        cv2.namedWindow("fm2 time", cv2.WINDOW_NORMAL)
        cv2.imshow("fm2 time", display)
        cv2.waitKey(10)

    def displayPath_over_time( self, path ):

        max = np.max( np.max( self.time ) )
        display = np.zeros( (len(self.speed), len(self.speed[0]), 3), np.uint8)
        for i in range(0, np.size( self.map, 0) ):
            for j in range(0, np.size( self.map, 1) ):
                val = (max - self.time[i][j]) / (max)
                shade = 255 - round( val*255 )
                display[i][j][0] = shade
                display[i][j][1] = shade
                display[i][j][2] = shade
    
        for p in path:
            cv2.circle( display, p, 1, [0,0,255], -1)

        cv2.circle( display, self.start, 10, 127, -1)
        cv2.circle( display, self.goal, 10, 127, -1)
    
        cv2.namedWindow("fm2 path over time", WINDOW_NORMAL)
        cv2.imshow("fm2 path over time", display)
        cv2.waitKey(10)

    def displayPath_over_map( self, path ):

        display = np.zeros( (len(self.speed), len(self.speed[0]), 3), np.uint8)
        for i in range(0, np.size( self.map, 0) ):
            for j in range(0, np.size( self.map, 1) ):
                if self.map[i][j] == 255:
                    display[i][j][0] = 0
                    display[i][j][1] = 0
                    display[i][j][2] = 0
                else:
                    display[i][j][0] = 255
                    display[i][j][1] = 255
                    display[i][j][2] = 255
                
        for p in path:
            cv2.circle( display, (round(p[1]), round(p[0])), 2, (0,0,255), -1)

        cv2.circle( display, (self.start[0], self.start[1]), 5, (0,0,255), -1)
        cv2.circle( display, (self.goal[0], self.goal[1]), 5, (0,0,255), -1)
    
        cv2.namedWindow("fm2 path over map", cv2.WINDOW_NORMAL)
        cv2.imshow("fm2 path over map", display)
        cv2.waitKey(10)

    def displayPath_over_speedMap( self, path ):

        max = np.max( np.max( self.time ) )
        display = np.zeros( (len(self.speed), len(self.speed[0]), 3), np.uint8)
        for i in range(0, np.size( self.map, 0) ):
            for j in range(0, np.size( self.map, 1) ):
                val = (max - self.speed[i][j]) / (max)
                shade = 255 - round( val*255 )
                display[i][j][0] = shade
                display[i][j][1] = shade
                display[i][j][2] = shade
    
        for p in path:
            cv2.circle( display, p, 1, [0,0,255], -1)

        cv2.circle( display, self.start, 10, 127, -1)
        cv2.circle( display, self.goal, 10, 127, -1)
    
        cv2.namedWindow("fm2 path over speed", WINDOW_NORMAL)
        cv2.imshow("fm2 path over speed", display)
        cv2.waitKey(10)

    def inflateWalls_to_crashRadius( self ):
    
        nbrs = [ [-1,0],[1,0],[0,-1],[0,1] ]

        s = 0
        while( s < self.crash_radius_c ):
            toInflate = np.zeros( ( np.size(self.map,0), np.size( self.map, 1) ), np.uint8)
            for i in range(1, np.size( self.map, 0)-1 ):
                for j in range(1, np.size( self.map, 1)-1 ):
                    if( self.speed[i][j] == 0.0):
                        for n in nbrs:
                            pp = [n[0] + i, n[1] + j]
                            toInflate[n[0]+i][n[1]+j] = 1
                        
            for i in range(0, np.size( self.map, 0) ):
                for j in range(0, np.size( self.map, 1) ):
                    if( toInflate[i][j] == 1):
                        self.speed[i][j] = 0.0

            s += 1 

    def inflateWalls_to_inflationRadius( self ):
        nbrs = [ [-1,0],[1,0],[0,-1],[0,1] ]
        inflateVal = 0.0
        inflateVal_prior = 0.0
        iters = self.inflation_radius_m / self.cell2Meter
        inflateIncrement = self.max_speed / iters

        while( inflateVal < self.max_speed ):
            inflateVal += inflateIncrement
            toInflate = np.zeros( ( np.size(self.map,0), np.size( self.map, 1) ), np.uint8)
            for i in range(1, np.size( self.map, 0)-1 ):
                for j in range(1, np.size( self.map, 1)-1 ):
                    if( self.speed[i][j] == inflateVal_prior):
                        for n in nbrs:
                            pp = [n[0] + i, n[1] + j]
                            toInflate[n[0]+i][n[1]+j] = 1
                        
            for i in range(0, np.size( self.map, 0) ):
                for j in range(0, np.size( self.map, 1) ):
                    if( toInflate[i][j] == 1 and self.speed[i][j] == self.max_speed):
                        self.speed[i][j] = inflateVal
        
            inflateVal_prior = inflateVal
