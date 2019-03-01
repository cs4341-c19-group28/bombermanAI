# This is necessary to find the main code
import sys
import random
import math

sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back
import random


class init():
    def __init__(self):
        self.dis = dis


inf = math.inf


class TestCharacter(CharacterEntity):
    search_level = 3
    self_loc = []
    isExp = False
    tick_bomb = 0

    def range(self, x, y, wrld):
        if (x > 0 and x < wrld.width()):
            if (y > 0 and y < wrld.height()):
                return True
        return False

    def wall_at(self, x, y, wrld):
        if (x > 0 and x < wrld.width()):
            if (y > 0 and y < wrld.height()):
                if not wrld.wall_at(x, y):
                    return True
        return False

    def DF_EXIT(self, wrld):
        exit_x, exit_y = wrld.exitcell
        self.self_loc[exit_x][exit_y] = 1000

    # returns all valid neighbors
    def get_neighbors(self, x, y, v, s):
        # List of empty cells
        cells = []
        # Go through neighboring cells
        if self.range(x + 1, y, s): cells.append((v, (x + 1, y)))
        if self.range(x - 1, y, s): cells.append((v, (x - 1, y)))
        if self.range(x, y + 1, s): cells.append((v, (x, y + 1)))
        if self.range(x, y - 1, s): cells.append((v, (x, y - 1)))
        if self.range(x + 1, y + 1, s): cells.append((v, (x + 1, y + 1)))
        if self.range(x + 1, y - 1, s): cells.append((v, (x + 1, y - 1)))
        if self.range(x - 1, y + 1, s): cells.append((v, (x - 1, y + 1)))
        if self.range(x - 1, y - 1, s): cells.append((v, (x - 1, y - 1)))

        return cells

    def search_close_monster(self, x, y, wrld):
        if (wrld.monsters_at(x + 1, y) or wrld.monsters_at(x + 2, y) or wrld.monsters_at(x - 1, y) or wrld.monsters_at(
                x - 2, y) or wrld.monsters_at(x, y + 1) or wrld.monsters_at(x, y + 2)
                or wrld.monsters_at(x, y - 1) or wrld.monsters_at(x, y - 2) or wrld.monsters_at(x + 1,
                                                                                                y + 1) or wrld.monsters_at(
                    x + 2, y + 2) or wrld.monsters_at(x + 1, y - 1) or wrld.monsters_at(x + 2, y - 2)
                or wrld.monsters_at(x - 1, y + 1) or wrld.monsters_at(x - 2, y + 2) or wrld.monsters_at(x - 1,
                                                                                                        y - 1) or wrld.monsters_at(
                    x - 2, y - 2) or wrld.monsters_at(x + 2, y) or wrld.monsters_at(x + 2, y + 1)
                or wrld.monsters_at(x + 2, y - 1) or wrld.monsters_at(x - 2, y + 1) or wrld.monsters_at(x - 2, y - 1)):
            return True
        return False

    def search_far_monster1(self, x, y, wrld):
        for dx in range(-3, 3, 1):
            for dy in range(-3, 3, 1):
                if (self.range(x + dx, y + dy, wrld)):
                    if (wrld.monsters_at(x + dx, y + dy)):
                        return True
            return False

    def search_far_monster2(self, x, y, wrld):
        # Avoid out-of-bounds access
        for dx in range(-4, 4, 1):
            # Avoid out-of-bounds access
            for dy in range(-4, 4, 1):
                if (self.range(x + dx, y + dy, wrld)):
                    if (wrld.monsters_at(x + dx, y + dy)):
                        return True
        return False

    def search_far_monster3(self, x, y, wrld):
        # Avoid out-of-bounds access
        for dx in range(-5, 5, 1):
            # Avoid out-of-bounds access
            for dy in range(-5, 5, 1):
                if (self.range(x + dx, y + dy, wrld)):
                    if (wrld.monsters_at(x + dx, y + dy)):
                        return True
        return False

    def dis_to_bomb(self, x, y, wrld):
        dis = []
        for dx in range(-5, 5, 1):
            if self.range(x + dx, y, wrld):
                if wrld.bomb_at(x + dx, y):
                    dis.append(abs(dx))
        for dy in range(-5, 5, 1):
            if self.range(x, y + dy, wrld):
                if wrld.bomb_at(x, y + dy):
                    dis.append(abs(dy))
        if len(dis) > 0:
            return min(dis)
        return 0

    def wall_monster_at(self, x, y, wrld):
        wall_monster = 0
        for dx in range(1, 5):
            if self.range(x + dx, y, wrld):
                if wrld.wall_at(x + dx, y): wall_monster += 1
                if wrld.monsters_at(x + dx, y): wall_monster += 1
            if self.range(x - dx, y, wrld):
                if wrld.wall_at(x - dx, y): wall_monster += 1
                if wrld.monsters_at(x - dx, y): wall_monster += 1
        for dy in range(1, 5):
            if self.range(x, y + dy, wrld):
                if wrld.wall_at(x, y + dy): wall_monster += 1
                if wrld.monsters_at(x, y + dy): wall_monster += 1
            if self.range(x, y - dy, wrld):
                if wrld.wall_at(x, y - dy): wall_monster += 1
                if wrld.monsters_at(x, y - dy): wall_monster += 1

        return wall_monster

    def get_score(self, wrld, loc):
        print("getting score")
        x = loc[0]
        y = loc[1]
        reward = 0
        reward += self.self_loc[x][y]
        # reward system
        if self.search_close_monster(x, y, wrld):
            reward += -500
        if self.search_far_monster1(x, y, wrld):
            reward += -300
        if self.search_far_monster2(x, y, wrld):
            reward += -100
        if self.search_far_monster3(x, y, wrld):
            reward += -50
        if wrld.monsters_at(x, y):
            reward += -1000
        if self.dis_to_bomb(x, y, wrld) > 0:
            reward += self.dis_to_bomb(x, y, wrld) * -500
        if wrld.explosion_at(x, y) is not None:
            reward += -1000
        if wrld.bomb_at(x, y) is not None:
            if wrld.bomb_at(x, y).timer < 3:
                reward += -1000
        print("LOC:", loc, "SCORE:", reward)
        return reward

    def expectimax_search(self, brd, search_level):
        c_level = 0
        search_result = 0
        max_val = -inf
        current_val = -inf

        for brd, val in self.look_for_cell(brd):
            current_val = max(current_val, self.es_val(brd, val, c_level + 1))
            if current_val > max_val:
                max_val = current_val
                search_result = val

        return search_result

    def max_value(self, brd, direction, location, level):  # val is a tuple of direction
        if level >= self.search_level:
            print("quiting search")
            return self.get_score(brd, (location[0] + direction[0], location[1] + direction[1]))
        value = -inf

        c = next(iter(brd.me().values()))

        for dx in [-1, 0, 1]:
            # Avoid out-of-bound indexing
            if (c.x + dx >= 0) and (c.x + dx < brd.width()):
                # Loop through delta y
                for dy in [-1, 0, 1]:
                    # Make sure the monster is moving
                    if (dx != 0) or (dy != 0):
                        # Avoid out-of-bound indexing
                        if (c.y + dy >= 0) and (c.y + dy < brd.height()):
                            # No need to check impossible moves
                            if not brd.wall_at(c.x + dx, c.y + dy):
                                # Set move in wrld
                                c.move(dx, dy)
                                # Get new world
                                (newwrld, events) = brd.next()

                                value=max(result[0], self.es_val(newwrld, (dx, dy), (c.x, c.y), level + 1))


        # for loc in self.look_for_cell(brd):
        #     print("searching at level", level)
        #     value = max(value, self.es_val(loc[0], loc[1], level + 1))
        #return value

    def es_val(self, brd, direction, location, level):
        if level >= self.search_level:
            print("quiting search")
            return self.get_score(brd, val)
        value = 0
        # m = next(iter(brd.me().values()))
        #
        # for dx in [-1, 0, 1]:
        #     # Avoid out-of-bound indexing
        #     if (c.x+dx >=0) and (c.x+dx < brd.width()):
        #         # Loop through delta y
        #         for dy in [-1, 0, 1]:
        #             # Make sure the monster is moving
        #             if (dx != 0) or (dy != 0):
        #                 # Avoid out-of-bound indexing
        #                 if (c.y+dy >=0) and (c.y+dy < brd.height()):
        #                     # No need to check impossible moves
        #                     if not brd.wall_at(c.x+dx, c.y+dy):
        #                         # Set move in wrld
        #                         c.move(dx, dy)
        #                         # Get new world
        #                         (newwrld,events) = brd.next()

        # for loc in self.look_for_cell(brd):
        #     print("searching at level", level)
        #      value = value + (self.max_value(loc[0], loc[1], level + 1))
        return value

    def isExp(self, wrld):
        for x in range(wrld.width()):
            for y in range(wrld.height()):
                if wrld.explosion_at(x, y):
                    return True
        return False

    def look_for_cell(self, brd):
        print("IN LOOK FOR CELL")
        cells = []
        if self.range(self.x + 1, self.y, brd):
            cells.append((brd, (self.x + 1, self.y)))
        if self.range(self.x - 1, self.y, brd):
            cells.append((brd, (self.x - 1, self.y)))
        if self.range(self.x, self.y + 1, brd):
            cells.append((brd, (self.x, self.y + 1)))
        if self.range(self.x, self.y - 1, brd):
            cells.append((brd, (self.x, self.y - 1)))
        if self.range(self.x + 1, self.y + 1, brd):
            cells.append((brd, (self.x + 1, self.y + 1)))
        if self.range(self.x + 1, self.y - 1, brd):
            cells.append((brd, (self.x + 1, self.y - 1)))
        if self.range(self.x - 1, self.y + 1, brd):
            cells.append((brd, (self.x - 1, self.y + 1)))
        if self.range(self.x - 1, self.y - 1, brd):
            cells.append((brd, (self.x - 1, self.y - 1)))
        cells.append((brd, (self.x, self.y)))
        print("cells", cells)

        return cells

    def append_list(self, wrld):
        cells = []
        mark = []
        value = 100
        exit_x, exit_y = wrld.exitcell
        cells = self.get_neighbors(exit_x, exit_y, value - 1, wrld)
        mark.append((exit_x, exit_y))
        for visted in cells:
            mark.append(visted[1])
        while cells:
            current_value, loc = cells[0]
            mark.append(loc)
            x, y = loc
            if wrld.wall_at(x, y):
                self.self_loc[x][y] = current_value - 5
                current_value -= 5
            else:
                self.self_loc[x][y] = current_value

            sub = self.get_neighbors(x, y, current_value - 1, wrld)
            for check in sub:
                if check[1] not in mark:
                    cells.append(check)
                    mark.append(check[1])
            cells.remove(cells[0])

    def do(self, wrld):

        if self.wall_monster_at(self.x, self.y, wrld) > 0:
            self.place_bomb()
        # init loc
        if self.self_loc == []:
            self.self_loc = [[0] * wrld.height() for i in range(wrld.width())]
            self.DF_EXIT(wrld)
            self.append_list(wrld)
        self.explosion = self.isExp(wrld)

        if self.explosion:
            self.tick_bomb += 1

        if self.tick_bomb == 5:
            self.append_list(wrld)
            self.explosion = False
            self.tick_bomb = 0

        loc = self.expectimax_search(wrld, self.search_level)
        dx = loc[0] - self.x
        dy = loc[1] - self.y
        print("MY LOC", self.x, self.y)
        self.move(dx, dy)  # execute our final decided on motion
