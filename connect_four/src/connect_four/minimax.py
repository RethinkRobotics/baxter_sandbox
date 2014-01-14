# Copyright (c) 2013, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import random

from copy import deepcopy


class BaxterAI(object):
    def __init__(self):
        """
        Interface class for a gripper on the Baxter Research Robot.

        @param gripper: robot limb <left/right> on which the gripper
                        is mounted
        """
        self._depth = 0

    def _move_possible(self, col, grid):
        return any(grid[i][col] == 0 for i in xrange(6))

    def _copy_move(self, col, grid, turn):
        cpy = deepcopy(grid)
        for i in xrange(6):
            if cpy[5 - i][col] == 0:
                cpy[5 - i][col] = turn
                return cpy

    def horz_count(self, grid, turn, row, col, consec, cont=True):
        cnt = 0
        for i in xrange(col, 7):
            if grid[row][i] == turn:
                cnt += 1
            elif cont == True or not grid[row][i] == 0:
                break
        if cnt >= consec:
            return True
        else:
            return False

    def vert_count(self, grid, turn, row, col, consec, cont=True):
        cnt = 0

        for i in xrange(row, 6):
            if grid[i][col] == turn:
                cnt += 1
            elif cont == True or not grid[i][col] == 0:
                break
        if cnt >= consec:
            return True
        else:
            return False

    def diag_count(self, grid, turn, row, col, consec, cont=True):
        r_cnt = 0
        j = col
        for i in xrange(row, 6):
            if grid[i][j] == turn:
                r_cnt += 1
                if r_cnt >= consec:
                    return True
            elif cont == True or not grid[i][j] == 0:
                break
            j += 1
            if j > 6:
                break

        l_cnt = 0
        j = col
        for i in xrange(row, -1, -1):
            if grid[i][j] == turn:
                l_cnt += 1
                if l_cnt >= consec:
                    return True
            elif cont == True or not grid[i][j] == 0:
                break
            j += 1
            if j > 6:
                break
        return False

    def _search(self, depth, grid, turn):
        ptl_move = []
        for col in xrange(7):
            if self._move_possible(col, grid):  # move is valid
                # make the move
                search_copy = self._copy_move(col, grid, turn)
                ptl_move.append(search_copy)

        if depth == 0 or len(ptl_move) == 0:
            # return the heuristic value found
            return self._value(grid)

        score = 0

        if turn == 1:
            opp_turn = 2
        else:
            opp_turn = 1
        for move in ptl_move:
            if move == None:
                break
            search_score = self._search(depth - 1, move, opp_turn)
            if search_score != None:
                score += search_score

        return -1 * score

    def _value(self, grid):
        '''
        Heuristic is:
        Number of 4-in-a-rows*20000
        + Number of 3-in-a-rows*10
        + Number of 2-in-a-rows
        - Number of opponent 4-in-a-rows*10000
        - Number of opponent 3-in-a-rows*10
        - Number of opponent 2-in-a-rows
        '''
        def streak(s_grid, turn, num):
            count = 0
            for i in xrange(6):
                for j in xrange(7):
                    if grid[i][j] == turn:
                        continuous = True
                        if num != 4:
                            continuous = False
                        count += int(self.horz_count(s_grid, turn, i, j,
                                                     num, cont=continuous))
                        count += int(self.vert_count(s_grid, turn, i, j,
                                                     num, cont=continuous))
                        count += int(self.diag_count(s_grid, turn, i, j,
                                                     num, cont=continuous))
            return count

        fours = streak(grid, 1, 4)
        threes = streak(grid, 1, 3)
        twos = streak(grid, 1, 2)
        opp_fours = streak(grid, 2, 4)
        opp_threes = streak(grid, 2, 3)
        opp_twos = streak(grid, 2, 2)

        score = 0

        if opp_fours > 0:
            return 500000
        else:
            return -1 * (fours * 100000 + threes * 100 + twos
                         - opp_threes * 100)
#
#         score = (fours * 12000 + threes * 10 + twos
#                  - opp_fours * 11000 - opp_threes * 10 - opp_twos)
#
#
#         return -1 * score

    def find_move(self, grid, depth, round):
        if round == 0:
            return 3
        self._depth = depth
        # check if move is possible for all columns
        good_moves = {}

        for col in xrange(7):
            if self._move_possible(col, grid):
                # play out move in current column
                grid_copy = self._copy_move(col, grid, 1)
                score = self._search(depth, grid_copy, 1)
                if self._depth % 2 == 0:
                    score = -1 * score
                good_moves[col] = score

        best_score = pow(-10, 21)
        best_move = None
        last_best = None
        changed = False
        moves = good_moves.items()
        print [score for move, score in moves]
        random.shuffle(moves)
        for move, score in moves:
            if score >= best_score:
                best_score = score
                best_move = move
            if move != 0 and last_best != score:
                changed = True
            last_best = best_score

        if changed == False:
            best_move = 3

        return best_move
