# import sys
# sys.path.insert(0, '..')
from entity import MonsterEntity
import random

class StupidMonster(MonsterEntity):
    """A pretty stupid monster"""
    def must_change_direction(self, wrld):
        # Get next desired position
        (nx, ny) = self.nextpos()
        # If next pos is out of bounds, must change direction
        if ((nx < 0) or (nx >= wrld.width()) or
            (ny < 0) or (ny >= wrld.height())):
            return True
            # If these cells are an explosion, a wall, or a monster, go away
            return (wrld.explosion_at(self.x, self.y) or
                    wrld.wall_at(nx, ny) or
                    wrld.monsters_at(nx, ny) or
                    wrld.exit_at(nx, ny))

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
                        if not wrld.wall_at(self.x + dx, self.y + dy):
                            cells.append((dx, dy))
        # All done
        return cells

    def do(self, wrld):
        """Pick an action for the monster"""
        if ((self.dx == 0 and self.dy == 0) or
            self.must_change_direction(wrld)):
            # Get list of safe moves
            safe = self.look_for_empty_cell(wrld)
        # Get list of safe moves
            safe = self.look_for_empty_cell(wrld)
        # Pick a move at random
            (dx, dy) = random.choice(safe)
            self.move(dx, dy)
