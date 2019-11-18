#!/usr/bin/env python

import rospy
from collections import deque
from sets import Set


class Frontier():
    '''
    Represents a frontier on a map. 
    A frontier is define as a region in which freespace cells border unknown cells. 
    A Frontier object contains a list of all known cells on a continuous boundary.
    Cells are represented by a tuple (x,y).
    '''
    def __init__(self, cells=[]):
        self._member_cells = cells


class FrontierCostmap():
    UNKNOWN = -1
    FREE = 0
    OCCUPIED = 100
    '''
    Represents a ROS Costmap. 
    Wraps a nav_msgs.OccupancyGrid message, and proides ease of use methods to work with the map
    '''
    def __init__(self, map_msg):
        self._width = map_msg.MapMetaData.width
        self._height = map_msg.MapMetaData.height
        self._resolution = map_msg.MapMetaData.resolution
        self._data = map_msg.data

    def _getIndex(self, x, y):
        '''
        Returns the data index that correponds to the given x, y coordinate.
        '''
        return (self._width * y) + x

    def isOnMap(self, x, y):
        '''
        Returns true iff the coordinates given are on the map
        '''
        return x >= 0 and y >= 0 and x < self._width and y < self._height

    def getCell(self, x, y):
        '''
        Returns the cell value at the given index if it exists.
        '''
        if not self.isOnMap(x, y):
            return None
        else:
            return self._data[self._getIndex(x, y)]

    def getNeighbors(self, x, y):
        '''
        Returns the 8 neighbors to the cell at x, y
        n n n  n = neighbor, p = (x, y) 
        n p n
        n n n
        '''
        neighbors = []
        if self.isOnMap(x - 1, y - 1): neighbors.append((x - 1, y - 1))
        if self.isOnMap(x, y - 1): neighbors.append((x, y - 1))
        if self.isOnMap(x + 1, y - 1): neighbors.append((x + 1, y - 1))
        if self.isOnMap(x + 1, y): neighbors.append((x + 1, y))
        if self.isOnMap(x + 1, y + 1): neighbors.append((x + 1, y + 1))
        if self.isOnMap(x, y + 1): neighbors.append((x, y + 1))
        if self.isOnMap(x - 1, y + 1): neighbors.append((x - 1, y + 1))
        if self.isOnMap(x - 1, y): neighbors.append((x - 1, y))
        return neighbors

    def isFrontier(self, x, y):
        '''
        returns true iff this cell is a frontier cell
        i.e. it is a known cell that is adjacent to a unknown cell
        '''
        if self.getCell(x, y) == self.FREE:
            return any(
                map(lambda p: p == self.UNKNOWN, self.getNeighbors(x, y)))
        else:
            return False

    def expandFrontier(self, x, y, visited_set):
        '''
        Returns the entire frontier boundary connected to the cell at x, y
        assumes that the cell at x, y is a frontier cell
        '''
        frontier_cells = [(x, y)]
        queue = deque()
        for p in self.getNeighbors(x, y):
            if self.isFrontier(*p) and p not in visited_set: queue.append(p)

        while len(queue) > 0:
            pos = queue.pop()
            frontier_cells.append(pos)
            visited_set.add(pos)
            for p in self.getNeighbors(*p):
                if self.isFrontier(*p) and p not in visited_set:
                    queue.append(p)
        return Frontier(frontier_cells)

    def findFrontiers(self, robot_x, robot_y):
        '''
        Finds frontiers starting at the given x, y grid. 
        Uses a breadth first search search outwards from the starting robot location.
        '''
        frontiers = []
        visited_set = Set()
        queue = deque()
        for p in self.getNeighbors(robot_x, robot_y):
            queue.append(p)

        while len(queue) > 0:
            pos = queue.pop()
            if pos in visited_set:
                continue
            else:
                visited_set.add(pos)
                if self.isFrontier(*pos):
                    frontiers.append(
                        self.expandFrontier(pos[0], pos[1], visited_set))
        return frontiers
