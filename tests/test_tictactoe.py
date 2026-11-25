import numpy as np

from robo_draw.core.tictactoe import best_move, occupied_cells, outcome


def test_robot_blocks_and_wins():
    almost = ["X", "X", ".", ".", ".", ".", ".", ".", "."]
    assert best_move(almost) == 2
    winning = ["O", "O", ".", "X", ".", ".", "X", ".", "."]
    assert best_move(winning) == 2
    assert outcome(["X", "X", "X", ".", ".", ".", ".", ".", "."]) == "X"
    assert outcome(["O", ".", ".", "O", ".", ".", "O", ".", "."]) == "O"
    assert outcome(["X", "O", "X", "X", "O", "O", "O", "X", "X"]) == "draw"


def test_camera_sees_one_mark_in_origin_cell():
    image = np.full((300, 300, 3), 255, dtype=np.uint8)
    image[230:270, 30:70] = 0
    flags, ratios = occupied_cells(image, ink_ratio=0.045)
    assert flags[0] is True
    assert ratios[0] > 0.045
    assert flags[1:] == [False] * 8
