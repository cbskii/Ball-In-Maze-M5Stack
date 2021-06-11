"""
This script generates a maze for the M5Stack along with a starting position for the ball.
"""

from ctypes import *
import textwrap

MAX_NUM_WALLS = 15
DISPLAY_WIDTH = 320
DISPLAY_HEIGHT = 240

# TODO PEP8 formatting
# TODO make executable shell script and able to run anywhere
ROOT_DIR = "../"
GENERATED_FILE_NAME = ROOT_DIR + "main/maze_generated.h"
GENERATED_FILE = (
"""\
/* THIS IS A GENERATED FILE, DO NOT EDIT MANUALLY. */

#ifndef MAZE_GENERATED_H
#define MAZE_GENERATED_H

#define MAX_NUM_WALLS ({})
#define GEN_DISPLAY_HEIGHT ({})
#define GEN_DISPLAY_WIDTH ({})

/* Initial ball position (ball_t) */
const char g_start_ball[] = {{\n{}\n}};

/* Generated maze (maze_t) */
const char g_generated_maze[] = {{\n{}\n}};

#endif /* MAZE_GENERATED_H */
"""
)


class VectorFloat(Structure):
    _fields_ = [("x", c_float),
                ("y", c_float),
                ("z", c_float)]


class Pos(Structure):
    _fields_ = [("x", c_uint16),
                ("y", c_uint16)]


class Wall(Structure):
    _fields_ = [("start", Pos),
                ("end", Pos)]


class Ball(Structure):
    _fields_ = [("prev_pos", Pos),
                ("pos", Pos),
                ("velocity", VectorFloat)]


class Maze(Structure):
    _fields_ = [("walls", Wall * MAX_NUM_WALLS),
                ("num_walls", c_uint8)]


def get_formatted_bytes(data):
    """
    Helper function to format a Python C struct as a series of indented bytes for header file
    generation.

    @return string of indented bytes
    """
    raw_str = str([hex(i) for i in bytes(data)])
    raw_str = raw_str.replace("'", "")
    raw_str = raw_str.replace("[", "")
    raw_str = raw_str.replace("]", "")
    raw_str = textwrap.fill(raw_str)
    raw_str = textwrap.indent(raw_str, '\t', lambda line: True)
    return raw_str


if __name__ == "__main__":
    # TODO test hardcoding same maze, eventually use DISPLAY_HEIGHT/DISPLAY_WIDTH
    ball = Ball(
            # Previous position
            (20, DISPLAY_HEIGHT // 2),

            # Current position (start)
            (20, DISPLAY_HEIGHT // 2),

            # Velocity
            ((0.0), (0.0), (0.0)))

    maze = Maze((
        # Walls made up of start and end positions
        ((0, 150), (90, 150)),
        ((160, 0), (160, 240))

        # number of walls
        ), 2)

    raw_ball_data_str = get_formatted_bytes(ball)
    raw_maze_data_str = get_formatted_bytes(maze)
    file_contents = GENERATED_FILE.format(MAX_NUM_WALLS, DISPLAY_HEIGHT, DISPLAY_WIDTH,
                                          raw_ball_data_str, raw_maze_data_str)

    with open(GENERATED_FILE_NAME, "w+") as f:
        f.write(file_contents)

