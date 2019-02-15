# This is necessary to find the main code
import sys
import random
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back
import random

class TestCharacter(CharacterEntity):

    def do(self, wrld):
        exit = [0, 0]
        self_x = wrld.me(self).x
        self_y = wrld.me(self).y
        for i in range(wrld.width()):
            for j in range(wrld.height()):
                if wrld.exit_at(i, j):
                    exit = [i, j]
        next_step = astar.op_path([self_x,self_y],exit,wrld)
        self.move(-self_x+next_step[0], -self_y+next_step[1])
