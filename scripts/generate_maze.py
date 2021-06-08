from ctypes import *

# TODO for now these are hardcoded, but would be good to share these values with the C header
MAX_NUM_WALLS = 15
DISPLAY_WIDTH = 320
DISPLAY_HEIGHT = 240


class Pos(Structure):
    _fields_ = [("x", c_ubyte),
                ("y", c_ubyte)]


class Wall(Structure):
    _fields_ = [("start", Pos),
                ("end", Pos)]


class Maze(Structure):
    _fields_ = [("walls", Wall * MAX_NUM_WALLS),
                ("num_walls", c_ubyte)]


if __name__ == "__main__":
    # TODO test hardcoding same maze
    maze = Maze((
        # Walls made up of start and end positions
        ((0, 150), (90, 150)),
        ((160, 0), (160, 240))

        # number of walls
        ), 2)

    data = str([hex(i) for i in bytes(maze)])
    data = data.replace("'", "")
    print(data)
