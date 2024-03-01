import curses

DISK_COLOR_MAP = {"red": 1, "green": 2, "blue": 3, "orange": 4} # color to size of disk map

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
    
class TowerOfHanoi:
    def __init__(self):
        # using a dictionary so we
        # can swap the towers around where we want

        self.towers = {} # str : Tower
        self.towers["A"] = Tower("A")
        self.towers["B"] = Tower("B")
        self.towers["C"] = Tower("C")

        for i in range(3, 0, -1):
            self.towers["A"].add_disk(i)
    
    def move_disk(self, source_tower : str, dest_tower : str):
        # Returns True if move is valid, False if move is invalid
        source : Tower = self.towers[source_tower]
        dest : Tower = self.towers[dest_tower]
        if len(source.disks) == 0:
            return False
        if len(dest.disks) == 0 or source.disks[-1] < dest.disks[-1]:
            dest.add_disk(source.remove_disk())
            return True
        return False
    
    def delete_tower(self, tower : str):
        # Returns True if tower is deleted, False if tower is invalid
        if tower in self.towers:
            del self.towers[tower]
            return True
        return False
    
    def add_tower(self, tower : str):
        # Returns True if tower is added, False if tower is invalid
        if tower not in self.towers:
            self.towers[tower] = Tower(tower)
            return True
        return False
    
    def get_optimal_move(self):
        # Returns the optimal move to solve the puzzle
        pass
    

class CursesUI:
    def __init__(self, game):
        self.game = game
        self.stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        self.stdscr.keypad(True)
        self.play()

    def draw(self):
        self.stdscr.clear()
        for tower_name, tower in self.game.towers.items():
            self.stdscr.addstr(f"{tower_name}:\n")
            for disk in tower.get_disks()[::-1]:
                self.stdscr.addstr("[" + "="*disk + "]\n")
        self.stdscr.refresh()

    def play(self):
        while True:
            self.draw()
            self.stdscr.addstr("Enter source tower and destination tower (e.g., AB): ")
            move = self.stdscr.getstr().decode('utf-8')
            if move == "q":
                break
            source, dest = move[0], move[1]
            if not self.game.move_disk(source, dest):
                print("wuttttt")
                self.stdscr.addstr("Invalid move!\n")
            # elif self.game.check_valid_state():
            #     self.stdscr.addstr("You won the game!\n")
            #     break
            self.stdscr.refresh()


    def cleanup(self):
        curses.nocbreak()
        self.stdscr.keypad(False)
        curses.echo()
        curses.endwin()

if __name__ == "__main__":
    game = TowerOfHanoi()
    ui = CursesUI(game)
    try:
        ui.play()
    finally:
        ui.cleanup()