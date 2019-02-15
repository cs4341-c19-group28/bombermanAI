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
        if self.size() != 0:
            return False
        else:
            return True

    def size(self):
        return len(self.queue)

    # for inserting an element in the queue
    def put(self, data, priority):
        self.queue.append((priority, data))
        # self.queue = self.queue[:index] + [newvalue] + self.queue[index+1:]
        self.queue.sort()

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
        cells = []
        # Go through neighboring cells
        for dx in [-1, 0, 1]:
            # Avoid out-of-bounds access
            if ((x + dx >= 0) and (x + dx < wrld.width())):
                for dy in [-1, 0, 1]:
                    # Avoid out-of-bounds access
                    if ((y + dy >= 0) and (y + dy < wrld.height())):
                        # Is this cell walkable?
                        # if not wrld.wall_at(x + dx, y + dy):#old
                        if not wrld.explosion_at(x + dx, y + dy) and not self.will_explode(x + dx, y + dy):
                            cells.append((x + dx, y + dy))
                            # All done
        return cells

    def astar(self, graph, start, goal):
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0
        while not frontier.empty():
            current_cost, current_loc = frontier.get()
            if current_loc == goal:
                break
            neighbors = self.get_neighbors(graph, current_loc[0], current_loc[1])
            for next in neighbors:
                # new_cost = cost_so_far[current] + graph.cost(current, next)
                if not graph.wall_at(next[0], next[1]):
                    graph_cost = graph.bomb_time
                elif graph.wall_at(next[0], next[1]):
                    graph_cost = graph.bomb_time + 30

                new_cost = current_cost + graph_cost
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current_loc
        return came_from, cost_so_far

    def smart_place_bomb(self, x, y, fuse_time):
        print("A",fuse_time,"b",self.fuse)
        if (self.fuse < 0):
            self.fuse = fuse_time + 2
            print("F",self.fuse)
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

        # (dx, dy) = random.choice(safe)
        path, cost = self.astar(wrld, (self.x, self.y), (wrld.exitcell))

        # todo remove debug prints in final sumbission
        print("fuse", self.fuse)
        print("will explode", self.explosion_loc)
        print("current loc", self.x, " ", self.y)
        print("path", path)

        self.set_cell_color(2, 2, " ")
        # for locations in path:
        #     loc =locations[0]
        #     self.set_cell_color(loc[0],loc[1]," ")
        # self.smart_place_bomb(self.x, self.y)

        # Iterate back to form a path
        step_list = [wrld.exitcell]

        # Go until we find None as our "came from"
        ok=True
        while ok:
            # Find where frontmost node came from
            try:
                came_from = path[step_list[0]]



                # If that position is None (IE the first move), break out of the while loop
                if came_from is None:
                    break
                else:
                    step_list = [came_from] + step_list
            except:
                print("ERROR: A* camefrom list invalid")
                (dx, dy) = random.choice(safe)
                ok=False



        # Get target x and y of next move
        if ok:
            tx, ty = step_list[1]

            # Compute deltas
            dx = tx - self.x
            dy = ty - self.y
        if (dx > 1):
            dx = 1
        if dy > 1:
            dy = 1
        if dx < -1:
            dx = 1
        if dy < -1:
            dy = -1
        print("next loc",self.x+dx,self.y+dy)
        if wrld.wall_at(self.x+dx, self.y+dy):
            self.smart_place_bomb(self.x, self.y, wrld.bomb_time)
            print("bombtime",wrld.bomb_time)

            if self.fuse >=-1:
                safe = self.look_for_empty_cell(wrld)
                (dx, dy) = random.choice(safe)

        # next_loc = path_element[0]
        self.update_fuse()
        self.move(dx, dy)
