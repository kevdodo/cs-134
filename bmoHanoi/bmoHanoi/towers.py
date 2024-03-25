import curses

from typing import List

DISK_COLOR_MAP = {5:"red", 4: "orange", 3:"yellow", 2:"green", 1: "blue", 9999: "black1", 9999: "black2", 9999: "black3"} 
COLOR_TO_DISK_MAP = {"blue":1, "green":2, "yellow":3, "orange":4, "red":5, "black1": 9999, "black2": 9999, "black3": 9999}

class Tower:
    def __init__ (self, name):
        self.name = name
        self.disks = [] # stack of disk sizes

    def get_disks(self):
        return self.disks
    
    def add_disk(self, disk_size : int):
        # Returns True if disk is added, False if disk is invalid
        if len(self.disks) > 0 and self.disks[-1] < disk_size:
            return False
        self.disks.append(disk_size)
        return True
    def remove_disk(self):
        # Returns disk if disk is removed, False if disk is invalid
        if len(self.disks) > 0:
            return self.disks.pop()
        return False

def check_tower_invalid(tower : Tower):
    '''
    We only have a few towers/disk, don't need to worry about 
    performance only needed to do when we actually have people
    messing with the towers
    '''
    for i in range(1, len(tower.disks)):
        if tower.disks[i-1] < tower.disks[i]:
            return True
    return False

    
class TowersOfHanoiSolver:
    def __init__(self):
        # using a dictionary so we
        # can swap the towers around where we want

        self.small_move = True
        self.top_colors = []
        # self.towers = {} # str : Tower
        # self.towers["A"] = Tower("A")
        # self.towers["B"] = Tower("B")
        # self.towers["C"] = Tower("C")

        # for i in range(3, 0, -1):
        #     self.towers["A"].add_disk(i)

    def update_solver(self, left_to_right_colors : list[str]):
        self.top_colors = left_to_right_colors

    def get_optimal_move(self):
        """Gets the right """
        if None in self.top_colors:
            return None, None

        if self.small_move:
            # get smallest, and move to the right
            donut_color = DISK_COLOR_MAP[min([COLOR_TO_DISK_MAP[color] for color in self.top_colors])]

            color_idx = self.top_colors.index(donut_color)

            peg_idx = (color_idx + 1) % 3
            peg_color = self.top_colors[peg_idx]
            self.small_move = False
        else:
            donut_color = sorted([COLOR_TO_DISK_MAP[color] for color in self.top_colors])
            second_smallest = DISK_COLOR_MAP[donut_color[1]]
            if second_smallest == 'black3':
                return None, None

            second_smallest_peg_idx = self.top_colors.index(second_smallest)
            smallest = DISK_COLOR_MAP[min([COLOR_TO_DISK_MAP[color] for color in self.top_colors])]
            smallest_peg_idx = self.top_colors.index(smallest)
            
            for i in range(3):
                if i != smallest_peg_idx and i != second_smallest_peg_idx:
                    peg_idx = i
                    peg_color = self.top_colors[i]
                    donut_color = second_smallest
            self.small_move = True


        # Donuts can't be black
        if donut_color in ['black1', 'black2', 'black3']:
            return None, peg_color


        return donut_color, peg_color
