# This is necessary to find the main code
import sys
import random

sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back
import random
# import Queue as queue
#
# pq=queue.PriorityQueue()


class PriorityQueue():
    def __init__(self):
        self.queue = []

    def __str__(self):
        return ' '.join([str(i) for i in self.queue])

    # for checking if the queue is empty
    def empty(self):
        if self.size()!=0:
            return False
        else:
            return True

    def size(self):
        return len(self.queue)
    # for inserting an element in the queue
    def put(self, data, priority):
        self.queue.append((priority,data))
        self.queue.sort()

        print("current queue",self.queue)

        # else:
        #     for index in range(0,self.size()):
        #         currentP=self.queue[index]
        #         print("VAL",currentP[0])
        #         if(priority<currentP[0]):
        #             self.queue.insert(index-1,(priority,data))
        #         else:
        #             continue


    # for popping an element based on Priority
    def get(self):
        print("quque",self.queue)
        if not self.empty():
            return self.queue.pop(0)
        else:
            print("ERROR: QUEUE EMPTY")



class TestCharacter(CharacterEntity):
    saved_bomb_loc = (None, None)
    fuse = -1
    explosion_loc = []

    # https://www.redblobgames.com/pathfinding/a-star/implementation.html
    def heuristic(self, a, b):
        (x1, y1) = a
        (x2, y2) = b
        return abs(x1 - x2) + abs(y1 - y2)

    def get_neighbors(self, wrld, x, y):
        # List of empty cells
        print("getting neighbors of ",x," ",y)
        cells = []
        # Go through neighboring cells
        for dx in [-1, 0, 1]:
            # Avoid out-of-bounds access
            if ((x + dx >= 0) and (x + dx < wrld.width())):
                for dy in [-1, 0, 1]:
                    # Avoid out-of-bounds access
                    if ((y + dy >= 0) and (y + dy < wrld.height())):
                        # Is this cell walkable?
                        if not wrld.wall_at(x + dx, y + dy):
                            cells.append((dx, dy))
                            # All done
        print("neighbors",cells)
        return cells

    def astar(self, graph, start, goal):
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0
        while not frontier.empty():
            print("-----------------")
            print("frontier", frontier)
            current_cost, current_loc = frontier.get()
            if current_loc == goal:
                break
            print("currentL", current_loc)
            neighbors = self.get_neighbors(graph, current_loc[0], current_loc[1])
            for next in neighbors:
                # new_cost = cost_so_far[current] + graph.cost(current, next)
                new_cost = current_cost + 1
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    print("adding ", next,"to frontier")
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current_loc

        print("A*Done")
        return came_from, cost_so_far

    def smart_place_bomb(self, x, y):
        if (self.fuse < 0):
            self.fuse = 12
            self.place_bomb()
            self.saved_bomb_loc = (self.x, self.y)
            self.explosion_loc.clear()
            for i in range(-4, 5):
                self.explosion_loc.append((x + i, y))
            for j in range(-4, 5):
                self.explosion_loc.append((x, y + j))

    def will_explode(self, x, y):
        if (self.fuse <= 2) and self.fuse != -1:
            if (x, y) in self.explosion_loc:
                return True
        return False

    def update_fuse(self):
        if (self.fuse >= -1):
            self.fuse = self.fuse - 1

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
                        if not wrld.wall_at(self.x + dx, self.y + dy) and not self.will_explode(self.x + dx,
                                                                                                self.y + dy) and not wrld.monsters_at(
                            self.x + dx, self.y + dy) and not wrld.explosion_at(self.x + dx, self.y + dy):
                            cells.append((dx, dy))
                            # All done
        return cells



    def do(self, wrld):
        safe = self.look_for_empty_cell(wrld)
        # Pick a move at random

        print("fuse", self.fuse)
        print("will explode", self.explosion_loc)
        print("current loc", self.x, " ", self.y)

        # (dx, dy) = random.choice(safe)
        path,cost=self.astar(wrld, (self.x, self.y), (wrld.exitcell))
        print("path",path)

        self.set_cell_color(2,2," ")
        # for locations in path:
        #     loc =locations[0]
        #     self.set_cell_color(loc[0],loc[1]," ")
        #self.smart_place_bomb(self.x, self.y)
        self.update_fuse()
        #self.move(1, 1)
