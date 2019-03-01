# This is necessary to find the main code
import sys
import random
import math

sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from sensed_world import SensedWorld
from colorama import Fore, Back
from math import sqrt
import random


class init():
    def __init__(self):
        self.dis = dis

inf = math.inf


class TestCharacter(CharacterEntity):
    search_level = 2
    self_loc = []
    isExp = False
    tick_bomb = 0
    #defind bound
    def range(self, x, y, wrld):
        if(x > 0 and x < wrld.width()):
            if (y > 0 and y < wrld.height()):
                return True
        return False
    #nowall
    def wall_at(self, x, y, wrld):
        if(x > 0 and x < wrld.width()):
            if (y > 0 and y < wrld.height()):
                if not wrld.wall_at(x, y):
                    return True
        return False
    #define exit
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
    #search monster within range 1-2
    def search_close_monster(self, x, y, wrld):
        if(wrld.monsters_at(x+1, y) or wrld.monsters_at(x+2, y) or wrld.monsters_at(x-1, y) or wrld.monsters_at(x-2, y) or wrld.monsters_at(x, y+1) or wrld.monsters_at(x, y+2)
        or wrld.monsters_at(x, y-1) or wrld.monsters_at(x, y-2) or wrld.monsters_at(x+1, y+1) or wrld.monsters_at(x+2, y+2) or wrld.monsters_at(x+1, y-1) or wrld.monsters_at(x+2, y-2)
        or wrld.monsters_at(x-1, y+1) or wrld.monsters_at(x-2, y+2) or wrld.monsters_at(x-1, y-1) or wrld.monsters_at(x-2, y-2) or wrld.monsters_at(x+2, y)or wrld.monsters_at(x+2, y+1)
         or wrld.monsters_at(x+2, y-1) or wrld.monsters_at(x-2, y+1) or wrld.monsters_at(x-2, y-1)):
            return True
        return False
    #search monster within range 3
    def search_far_monster1(self, x, y, wrld):
        for dx in range (-3,3,1):
            for dy in range(-3,3,1):
                if(self.range(x + dx, y + dy, wrld)):
                    if(wrld.monsters_at(x+dx, y+ dy)):
                        return True
            return False
    #search monster within range 4
    def search_far_monster2(self, x, y, wrld):
        # Avoid out-of-bounds access
        for dx in range (-4,4,1):
            # Avoid out-of-bounds access
            for dy in range(-4,4,1 ):
                if(self.range(x + dx, y + dy, wrld)):
                    if(wrld.monsters_at(x+dx, y+ dy)):
                        return True
        return False
    #search monster within range 5
    def search_far_monster3(self, x, y, wrld):
        # Avoid out-of-bounds access
        for dx in range (-5,5,1):
            # Avoid out-of-bounds access
            for dy in range(-5,5,1 ):
                if(self.range(x+dx, y+dy,wrld)):
                    if(wrld.monsters_at(x+dx,y+dy)):
                        return True
        return False
    #distance to bomb
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
    #wall or monster?
    def wall_monster_at(self, x, y, wrld):
        wall_monster = 0
        for dx in range(1, 5):
            if self.range(x + dx, y, wrld):
                if wrld.wall_at(x + dx, y):wall_monster += 1
                if wrld.monsters_at(x + dx, y):wall_monster += 1
            if self.range(x - dx, y, wrld):
                if wrld.wall_at(x - dx, y): wall_monster += 1
                if wrld.monsters_at(x - dx, y):wall_monster += 1
        for dy in range(1, 5):
            if self.range(x, y + dy, wrld):
                if wrld.wall_at(x, y + dy):wall_monster += 1
                if wrld.monsters_at(x, y + dy):wall_monster += 1
            if self.range(x, y - dy, wrld):
                if wrld.wall_at(x, y - dy):wall_monster += 1
                if wrld.monsters_at(x, y - dy):wall_monster += 1
        
        return wall_monster
    #prob
    #def monster_to_me(self,wrld,loc):
    #Sensed world mon
    
    #def get_monster_action(self,wrld,loc):
    #newWorld = SensedWorld.from_world(wrld)
    #m = SensedWorld.monsters_at(x,y)
    #monster.move(action1[],action2[])
    #cells.append(newWorld,action)
    


    #reward system
    def get_score(self, wrld, loc):
        x = loc[0]
        y = loc[1]
        reward = 0
        reward += self.self_loc[x][y]
        #reward system
        #if self.search_close_monster(x, y, wrld):reward += -500
        #if self.search_far_monster1(x,y, wrld):reward += -300
        #if self.search_far_monster2(x,y, wrld):reward += -100
        #if self.search_far_monster3(x,y, wrld):reward += -50
        #if wrld.monsters_at(x,y):reward += -1000
        #if self.dis_to_bomb(x,y,wrld) > 0:reward += self.dis_to_bomb(x,y,wrld) * -500
        #if wrld.explosion_at(x, y) is not None:reward += -1000
        #if wrld.bomb_at(x,y) is not None:
        #if wrld.bomb_at(x,y).timer < 3:reward += -1000
        Pro_kill = self.avoid_mon(wrld,(x,y))
        if Pro_kill[1]:
            if(Pro_kill[1]<9):
                if(Pro_kill[1]<9):
                    reward -=50
                if(Pro_kill[1]<8):
                    reward -=100
                if(Pro_kill[1]<7):
                    reward -=500
                if(Pro_kill[1]<6):
                    reward -=700
                if(Pro_kill[1]<5):
                    reward -=1000
                if(Pro_kill[1]<4):
                    reward -=1300
                if(Pro_kill[1]<3):
                    reward -=1500
                if(Pro_kill[1]<2):
                    reward -=2000
            else:
                reward -= (1/Pro_kill[1])*2000

        if wrld.monsters_at(x,y):
            reward -=1000
        if self.isEmpty(x,y,wrld)<2:
            reward -=600
        if self.isEmpty(x,y,wrld)<5:
            reward -= 300
        if self.dis_to_bomb(x,y,wrld) > 0:
            reward -=(5/self.dis_to_bomb(x,y,wrld))*500
        if wrld.bomb_at(x,y) is not None:
            if wrld.bomb_at(x,y).timer <=2:
                reward -=10000
        if wrld.explosion_at(x,y) is not None:
            reward -= 10000
        if wrld.exit_at(x, y):
            reward += 100000
        return reward

    #calculate the distance bettween character position to monster position
    def dis_to_mon(self,wrld,cp,mp):
        x1, y1 = cp
        x2, y2 = mp
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        # d=math.sqrt(math.pow(abs(x1 - x2),2) + math.pow(abs(y1 - y2),2))
        #if dx <=2 and dy <=2:
        #score -= 10000
        d = sqrt((dx*dy)+(dx*dy))
        if(d==0):
            return 1
        return 1/d
        
#def value(s)
#if s is a max node return maxValue(s)
#if s is an exp node return expValue(s)
#if s is a terminal node return evaluation(s)
#Event?
    def expectimax_search(self, wrld, search_level):
        #for Event in event:
        c_level = 0
        search_result = 0
        max_val = -inf
        current_val = -inf
        
        #me_loc = next(iter(wrld.characters.values()))
        #val is a tuple
        for wrld, val in self.look_for_cell(wrld):
            newWorld = SensedWorld.from_world(wrld)
            character = newWorld.me(self)
            character.x = val[0]
            character.y = val[1]
            current_val = max(current_val, self.expValue(wrld, val, c_level + 1))
            if current_val > max_val:
                max_val = current_val
                search_result = val
    
        return search_result
#def maxValue(s)
#values = [value(s’) for s’ in successors(s)]
#return max(values)
    def maxValue(self, wrld, val, level):
        if level >= self.search_level:
            return self.get_score(wrld, val)
        value = -inf
        for loc in self.look_for_cell(wrld):
            newWorld = SensedWorld.from_world(loc[0])
            character = newWorld.me(self)
            character.x = loc[1][0]
            character.y = loc[1][1]
            value = max(value, self.expValue(newWorld, loc[1], level + 1))
        return value
        
    def fmax_value(self, wrld, direction, location, level):  # val is a tuple of direction
        if level >= self.search_level:
            print("quiting search")
            return self.get_score(wrld, (location[0] + direction[0], location[1] + direction[1]))
        value = -inf
        
        c = next(iter(wrld.me().values()))
        
        for dx_c in [-1, 0, 1]:
            # Avoid out-of-bound indexing
            if (c.x + dx >= 0) and (c.x + dx < wrld.width()):
                # Loop through delta y
                for dy in [-1, 0, 1]:
                    # Make sure the monster is moving
                    if (dx != 0) or (dy != 0):
                        # Avoid out-of-bound indexing
                        if (c.y + dy >= 0) and (c.y + dy < wrld.height()):
                            # No need to check impossible moves
                            if not wrld.wall_at(c.x + dx, c.y + dy):
                                # Set move in wrld
                                c.move(dx, dy)
                                # Get new world
                                (newwrld, events) = wrld.next()
                                    #value=max(result[0], self.es_val(newwrld, (dx, dy), (c.x, c.y), level + 1))
        return value

#def expValue(s)
#values = [value(s’) for s’ in successors(s)]
#weights = [probability(s, s’) for s’ in successors(s)]
#return expectation(values, weights)
    def expValue(self, wrld, val, level):
        #if terminal
        if level >= self.search_level:
            return self.get_score(wrld, val)
        value = 0
        mon = self.avoid_mon(wrld,val)
        if not mon[0]:
            value = value + (self.maxValue(wrld,val, level + 1))
            return value
        mon_val = self.look_for_monster(wrld,mon[0][1])
        for vals in mon_val:
            vals[0].next()
            pb = self.dis_to_mon(vals[0],val,vals[1])
            value = value + ((pb)*self.maxValue(vals[0],val, level + 1))
        return value

    def isExp(self, wrld):
        for x in range(wrld.width()):
            for y in range(wrld.height()):
                if wrld.explosion_at(x, y):
                    return True
        return False
    
    def isEmpty(self,x,y,wrld):
        empty = 0
        for dx in range(-1, 2, 1):
            for dy in range(-1, 2 ,1):
                if(self.range(x + dx, y + dy, wrld)):
                    if(wrld.empty_at(x + dx, y + dy)):
                        empty += 1
        return empty
    #avoid monster, what is the chance to be hit
    def avoid_mon(self,wrld,p):
        all_mon = []
        near_mon = 0
        disToMon = 0
        m1 = p[0]
        m2 = p[1]
        for x in range(wrld.width()):
            for y in range(wrld.height()):
                monsters = wrld.monsters_at(x,y)
                if monsters:
                    for mon in monsters:
                        all_mon.append((mon,(x,y)))#get all monster
        for mon in all_mon:
            dx = abs(x - mon[1][0])
            dy = abs(y - mon[1][1])
            if (near_mon == 0):
                near_mon = mon
                disToMon = sqrt((m1 - mon[1][0])*(m1 - mon[1][0]) + (m2 - mon[1][1])*(m2 - mon[1][1]))
            else:
                dis  = sqrt((m1 - mon[1][0])*(m1 - mon[1][0]) + (m2 - mon[1][1])*(m2 - mon[1][1]))
                if dis < disToMon:
                    near_mon = mon
                    disToMon = dis

        return near_mon, disToMon
    # 8 near safe cells
    def look_for_cell(self, brd):
        cells = []
        if self.wall_at(self.x + 1, self.y, brd):
            cells.append((brd, (self.x + 1, self.y)))
        if self.wall_at(self.x - 1, self.y, brd):
            cells.append((brd, (self.x - 1, self.y)))
        if self.wall_at(self.x, self.y + 1, brd):
            cells.append((brd, (self.x, self.y + 1)))
        if self.wall_at(self.x, self.y - 1, brd):
            cells.append((brd, (self.x, self.y - 1)))
        if self.wall_at(self.x + 1, self.y + 1, brd):
            cells.append((brd, (self.x + 1, self.y + 1)))
        if self.wall_at(self.x + 1, self.y - 1, brd):
            cells.append((brd, (self.x + 1, self.y - 1)))
        if self.wall_at(self.x - 1, self.y + 1, brd):
            cells.append((brd, (self.x - 1, self.y + 1)))
        if self.wall_at(self.x - 1, self.y - 1, brd):
            cells.append((brd, (self.x - 1, self.y - 1)))
        cells.append((brd, (self.x,self.y)))
        
        return cells
    def flook_for_monster(self,wrld,loc):
        x,y = loc
        cells = []
        if self.wall_at(self.x + 1, self.y, wrld):
            newWorld = SensedWorld.from_world(wrld)
            mon = newWorld.monsters_at(x,y)
            mon.move(self.x,self.y)
            cells.append((newWorld, (self.x + 1, self.y)))
        if self.wall_at(self.x - 1, self.y, wrld):
            newWorld = SensedWorld.from_world(wrld)
            mon = newWorld.monsters_at(x,y)
            mon.move(self.x,self.y)
            cells.append((newWorld, (self.x - 1, self.y)))
        if self.wall_at(self.x, self.y + 1, wrld):
            newWorld = SensedWorld.from_world(wrld)
            mon = newWorld.monsters_at(x,y)
            mon.move(self.x,self.y)
            cells.append((newWorld, (self.x, self.y + 1)))
        if self.wall_at(self.x, self.y - 1, wrld):
            newWorld = SensedWorld.from_world(wrld)
            mon = newWorld.monsters_at(x,y)
            mon.move(self.x,self.y)
            cells.append((newWorld, (self.x, self.y - 1)))
        if self.wall_at(self.x + 1, self.y + 1, wrld):
            newWorld = SensedWorld.from_world(wrld)
            mon = newWorld.monsters_at(x,y)
            mon.move(self.x,self.y)
            cells.append((newWorld, (self.x + 1, self.y + 1)))
        if self.wall_at(self.x + 1, self.y - 1, wrld):
            newWorld = SensedWorld.from_world(wrld)
            mon = newWorld.monsters_at(x,y)
            mon.move(self.x,self.y)
            cells.append((newWorld, (self.x + 1, self.y - 1)))
        if self.wall_at(self.x - 1, self.y + 1, wrld):
            newWorld = SensedWorld.from_world(wrld)
            mon = newWorld.monsters_at(x,y)
            mon.move(self.x,self.y)
            cells.append((newWorld, (self.x - 1, self.y + 1)))
        if self.wall_at(self.x - 1, self.y - 1, wrld):
            newWorld = SensedWorld.from_world(wrld)
            mon = newWorld.monsters_at(x,y)
            mon.move(self.x,self.y)
            cells.append((newWorld, (self.x - 1, self.y - 1)))
        return cells
    #moves of monster in sensed world
    def look_for_monster(self,wrld,loc):
        cells = []
        x, y = loc
        Eight_move = [(x, y - 1), (x, y + 1),(x + 1, y - 1),(x - 1, y), (x + 1, y),(x - 1, y + 1),(x + 1, y + 1),(x - 1, y - 1)]
        for action in Eight_move:
            if self.wall_at(action[0], action[1], wrld):
                newWorld = SensedWorld.from_world(wrld)
                monster = newWorld.monsters_at(x, y)[0]
                monster.move(action[0], action[1])
                cells.append((newWorld, action))

        return cells
    #visted?
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
 
 #if self.wall_monster_at(self.x, self.y, wrld) > 0:
 #self.place_bomb()
        #init loc
        if self.self_loc == []:
            self.self_loc = [[0] * wrld.height() for i in range(wrld.width())]
            self.DF_EXIT(wrld)
            self.append_list(wrld)
        self.explosion = self.isExp(wrld)
            
        if self.explosion:
            self.tick_bomb += 1
        self.place_bomb()
        
        if self.tick_bomb == 3:
            self.append_list(wrld)
            self.explosion = False
            self.tick_bomb = 0
        
        loc= self.expectimax_search(wrld, self.search_level)
        dx = loc[0] - self.x
        dy = loc[1] - self.y
        self.move(dx, dy) # execute our final decided on motion
        





