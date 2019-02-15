# This is necessary to find the main code
import sys
import random

sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back
import random

class TestCharacter(CharacterEntity):
    saved_bomb_loc = (None, None)
    fuse = -1
    explosion_loc = []

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
        if (self.fuse <=2)and self.fuse!=-1:
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
                                                                                                self.y + dy) and not wrld.monsters_at(self.x+dx,self.y+dy)and not wrld.explosion_at(self.x+dx,self.y+dy):
                            cells.append((dx, dy))
                            # All done
        return cells

    def do(self, wrld):
        safe = self.look_for_empty_cell(wrld)
        # Pick a move at random

        print("fuse",self.fuse)
        print("will explode",self.explosion_loc)
        print("current loc",self.x, " ",self.y)

        (dx, dy) = random.choice(safe)
        self.smart_place_bomb(self.x, self.y)
        self.update_fuse()
        self.move(dx, dy)

