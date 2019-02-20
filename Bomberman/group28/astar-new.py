import sys
import random
import queue

# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back
import random
import math

# https://www.geeksforgeeks.org/priority-queue-in-python/

class PriorityQueue():
    def __init__(self):
        self.queue = []
    
    def __str__(self):
        return ' '.join([str(i) for i in self.queue])
    
    # for checking if the queue is empty
    def isEmpty(self):
        return len(self.queue) == []
    
    # for inserting an element in the queue
    def put(self,start,data):
        self.queue.append(data)
    
    # for popping an element based on Priority
    def get(self):
        try:
            max = 0
            for i in range(len(self.queue)):
                if self.queue[i] > self.queue[max]:
                    max = i
            item = self.queue[max]
            del self.queue[max]
            return item
        except IndexError:
            print()
            exit()

# class graph():
#
#     def neighbors(self,loc):
#
#     def cost(self,loc):
def look_for_empty_cell(self, wrld):
    # List of empty cells
    cells = []
    # Go through neighboring cells
    for dx in [-1, 0, 1]:
        # Avoid out-of-bounds access
        if ((self.x + dx >= 0) and (self.x + dx < wrld.width())):
            for dy in [-1, 0, 1]:
                # Avoid out-of-bounds access
                if ((self.y + dy >= 0) and (self.y + dy < wrld.height())):
                    # Is this cell walkable?
                    if not wrld.wall_at(self.x + dx, self.y + dy) or wrld.explosion_at(self.x + dx, self.y + dy) or wrld.monsters_at(self.x + dx, self.y + dy):
                        cells.append((dx, dy))
        # All done
    return cells

def heuristic(next, target):
    nexts = next.current
    targets = target.current
    return abs(nexts[0] - targets[0]) + abs(nexts[1]-targets[1])

def astar(start, goal, world):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    nexts = PriorityQueue(start)
    targets = PriorityQueue(target)
        
    while not frontier.empty():
        current = frontier.get()
            
        if current == goal:
            break
        neighbors = self.look_for_empty_cell(world)
        for next in neighbors:
            new_cost = cost_so_far[current] + 1
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + self.distance(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
     


def op_path(start, goal, world):
    foundPath = astar(start, goal, world)
    nextPath = foundPath

    while foundPath.current is not start:
        nextPath = foundPath.current
        foundPath = foundPath.came_from
    return nextPath


