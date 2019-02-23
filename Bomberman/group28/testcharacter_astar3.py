# This is necessary to find the main code
import sys
import random
import math

sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back
import random

'''A priority queue class for A* to use'''


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
        print("ADDING TO QUEUE", (priority, data))
        self.queue.sort()  # TODO put items in queue more inteligently if we have time

    def get(self):
        if not self.empty():
            return self.queue.pop(0)
            print("QUEUE", self.queue)

        else:
            print("ERROR: QUEUE EMPTY")



class TestCharacter(CharacterEntity):
    saved_bomb_loc = (None, None)
    fuse = -1
    explosion_loc = []

    # return the '' dist '' between two tuples
    def distance(self, a, b):
        (x1, y1) = a
        (x2, y2) = b
        d = abs(x1 - x2) + abs(y1 - y2)
        # d=math.sqrt(math.pow(abs(x1 - x2),2) + math.pow(abs(y1 - y2),2))
        print("distance", a, b, d)
        return d

    # returns all valid neighbors
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
                        if not wrld.explosion_at(x + dx, y + dy):
                            cells.append((x + dx, y + dy))
                            # All done
        return cells

    # return true if a monster is present within a certain radius of a location
    def monseter_search(self, wrld, x, y, radius):
        # Go through neighboring cells
        print("monster searching")
        for dx in range(-radius, radius):
            # Avoid out-of-bounds access
            if ((x + dx >= 0) and (x + dx < wrld.width())):
                for dy in range(-radius, radius):
                    # Avoid out-of-bounds access
                    if ((y + dy >= 0) and (y + dy < wrld.height())):
                        print("searching cell,", x + dx, y + dy)
                        if wrld.monsters_at(x + dx, y + dy):
                            print("returining ture")
                            return True
        return False

    # Returns x, y coordinate of monster
    def monseter_search_2(self, wrld, x, y, radius):
        # Go through neighboring cells
        for dx in range(-radius, radius):
            # Avoid out-of-bounds access
            if ((x + dx >= 0) and (x + dx < wrld.width())):
                for dy in range(-radius, radius):
                    # Avoid out-of-bounds access
                    if ((y + dy >= 0) and (y + dy < wrld.height())):
                        if wrld.monsters_at(x + dx, y + dy):
                            return (x + dx, y + dy)

    # an implementation of A* takes in the world graph from bomberman, start(tuple), goal(tuple) and returns a dict of came from locations
    def astar(self, graph, start, goal):
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():  # while I am not done
            current_cost, current_loc = frontier.get()  # get the lowest scored value from the priority queue
            if current_loc == goal:  # if done break
                break
            neighbors = self.get_neighbors(graph, current_loc[0],
                                           current_loc[1])  # find all valid neighbors for the current cell
            for next in neighbors:  # for each neigbor

                # calculate the cost of moving to the neighbor
                if not graph.wall_at(next[0], next[1]):
                    graph_cost = 1
                elif graph.wall_at(next[0], next[1]):
                    graph_cost = graph.bomb_time + 20
                # if self.monseter_search(graph, next[0], next[1], 5):
                #     graph_cost = 60
                # if self.monseter_search(graph, next[0], next[1], 2):
                #     graph_cost = 80
                # if self.will_explode(next[0], next[1]):
                #     graph_cost = 80

                new_cost = current_cost + graph_cost  # sum the cost to get here
                if next not in cost_so_far or new_cost < cost_so_far[
                    next]:  # if no value saved for the cell or a better value found
                    cost_so_far[next] = new_cost  # save the value
                    priority = new_cost + self.distance(goal, next)  # f=g+h
                    frontier.put(next,
                                 priority)  # put the value into the queue as a tuple of a loc tuple and the prioity rank in the queue
                    came_from[next] = current_loc  # update the path dictionary
        return came_from, cost_so_far

    # Returns x, y coordinate that goes away from monster
    def avoid_mon(self, graph):
        if self.monseter_search(graph, self.x, self.y, 3):
            mon_at = self.monseter_search_2(graph, self.x, self.y, 3)
            relative_mon = (mon_at[0] - self.x, mon_at[1] - self.y)
            init_dest = (-1 * relative_mon[0], -1 * relative_mon[1])
            if init_dest[0] != 0:
                init_dest = (init_dest[0] / abs(init_dest[0]), init_dest[1])
            if init_dest[1] != 0:
                init_dest = (init_dest[0], init_dest[1] / abs(init_dest[1]))

            if (self.x + init_dest[0]) < 0:
                init_dest = (init_dest[0] + 1, init_dest[1])
            elif (self.x + init_dest[0]) >= graph.width():
                init_dest = (init_dest[0] - 1, init_dest[1])
            if (self.y + init_dest[1]) < 0:
                init_dest = (init_dest[0], init_dest[1] + 1)
            elif (self.y + init_dest[1]) >= graph.height():
                init_dest = (init_dest[0], init_dest[1] - 1)
            projected_dest = (self.x + init_dest[0], self.y + init_dest[1])
            return projected_dest
        safe = self.look_for_empty_cell(graph)
        (dx, dy) = random.choice(safe)
        return (self.x + dx, self.y + dy)
        # Returns x, y coordinate that goes away from monster

    def avoid_mon2(self, graph, radius):
        print("avoiding mon2")
        if self.monseter_search(graph, self.x, self.y, radius):
            monster_loc = self.monseter_search_2(graph, self.x, self.y, radius)
            safe = self.look_for_empty_cell(graph)
            frontier = PriorityQueue()
            # frontier.put((self.x, self.y), 0)
            for loc in safe:
                new_loc = (self.x + loc[0], self.y + loc[1])
                if(new_loc==(self.x,self.y)):
                    frontier.put(new_loc, 999 )
                else:
                    threat_score=-1*self.distance(new_loc, monster_loc)
                    goal_score=self.distance(new_loc,graph.exitcell)
                    score=threat_score+goal_score
                    frontier.put(new_loc, (score ))
            print("Frontier")

            current_cost, current_loc = frontier.get()

            return (current_loc)
        else:
            print("ERROR: No monster found")
            return(self.x,self.y)

    # function to place a bomb but also save the predicted explosion location
    def smart_place_bomb(self, x, y, fuse_time):
        if (self.fuse < 0):
            self.fuse = fuse_time + 1
            self.place_bomb()
            self.saved_bomb_loc = (self.x, self.y)
            self.explosion_loc.clear()
            for i in range(-4, 5):
                self.explosion_loc.append((x + i, y))
            for j in range(-4, 5):
                self.explosion_loc.append((x, y + j))

    # function to check if a cell will explode next turn
    def will_explode(self, x, y):
        if (self.fuse <= 2) and self.fuse != -1:
            if (x, y) in self.explosion_loc:
                return True
        return False

    # counts down the fuse, must be called each turn
    def update_fuse(self):
        if (self.fuse >= -1):
            self.fuse = self.fuse - 1
        if (self.fuse == -2):
            self.explosion_loc.clear()  # bug fix for list not clearing properly

    # return a list of neighbors that are valid moves and don't kill you
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
        dx = 0
        dy = 0
        monster_search_rad = 3

        if self.fuse > 0:
            jc_ABOMB = True
        else:
            jc_ABOMB = False

        if self.monseter_search(wrld, self.x, self.y, monster_search_rad):  # behavior when the fuse is running
            print("--AVOIDING MONSTER")
            # safe = self.look_for_empty_cell(wrld)
            # (dx, dy) = random.choice(safe)
            target = self.avoid_mon2(wrld, monster_search_rad)
            print("Target = ", target)
            path, cost = self.astar(wrld, (self.x, self.y), target)
            step_list = [target]
        elif (jc_ABOMB):
            print("--RANDOM")
            if self.will_explode(self.x + dx, self.y + dy):  # never step into an explosion
                safe = self.look_for_empty_cell(wrld)
                (dx, dy) = random.choice(safe)
        else:
            print("--A*")
            path, cost = self.astar(wrld, (self.x, self.y), (wrld.exitcell))  # do A*
            step_list = [wrld.exitcell]

            # TODO remove debug prints in final sumbission
            print("path", path)

        print("fuse", self.fuse)
        print("will explode", self.explosion_loc)
        print("current loc", self.x, " ", self.y)
        # TODO remove debug prints in final sumbission

        # turn dict into a list
        # step_list = [wrld.exitcell]

        # Go until we find None as our "came from"
        ok = True

        while ok:
            try:
                # Find where frontmost node came from
                came_from = path[step_list[0]]
                print("working",came_from)
                # If that position is None (IE the first move), break out of the while loop
                if came_from is None:
                    break
                else:
                    step_list = [came_from] + step_list
            except:  # if A* doesn't find a solution
                ok = False
                break

        if ok:
            tx, ty = step_list[1]

            # Compute deltas
            dx = tx - self.x
            dy = ty - self.y

        # make sure dx dy are valid
        if (dx > 1):
            dx = 1
        if dy > 1:
            dy = 1
        if dx < -1:
            dx = 1
        if dy < -1:
            dy = -1

        # place a bomb if pathing through a wall
        if wrld.wall_at(self.x + dx, self.y + dy):
            self.smart_place_bomb(self.x, self.y, wrld.bomb_time)

        # Done in avoid_mon
        # if self.fuse > 0 and not self.monseter_search(wrld, self.x, self.y, 2):
        #     if self.will_explode(self.x + dx, self.y + dy):  # never step into an explosion
        #         safe = self.look_for_empty_cell(wrld)
        #         (dx, dy) = random.choice(safe)

        self.update_fuse()  ##runs the count down each turn
        self.move(dx, dy)  # execute our final decided on motion
