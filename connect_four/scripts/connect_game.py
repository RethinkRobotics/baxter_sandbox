#!/usr/bin/python

"""
Baxter RSDK Connect 4
"""

import argparse
import sys

from copy import deepcopy
from math import pow
from os import system

import rospy

from std_msgs.msg import String

import baxter_interface

import connect_four


class ConnectFour(object):
    def __init__(self, limb, color):
        # grid state 6 rows, 7 cols
        # 0 unoccupied
        # 1 baxter owned
        # 2 user owned
        if color == 'red':
            self.baxter_color = 1
            self.user_color = 2
        else:
            self.baxter_color = 2
            self.user_color = 1

        self.grid = [[0 for _i in range(7)] for _j in range(6)]
        for i in xrange(len(self.grid)):
            print self.grid[i]
        self._perceived_grid = deepcopy(self.grid)
        self._last_grid = None

        self.game_over = False
        self.winner = 0
        self.user = None  # user name (string)
        self._turn = 0  # 1 baxter, 2 user
        self._depth = 0  # Depth of minimax search [1(easy)-4(hard)]
        self._round = 0  # Number of turns played
        self._user_pieces = 0  # Number of user pieces played
        self._robot_pieces = 0  # Number of robot pieces played
        self._total_pieces = 0
        self._robot_cnt = 0
        self._user_cnt = 0
        self._baxter_cnt = 0
        self._pieces = 0  # Number of measured pieces

        grid_state_topic = '/vision/connect_four_state'
        _grid_state_sub = rospy.Subscriber(
            grid_state_topic,
            String,
            self._on_grid_state)

        err_msg = ("Connect Four failed waiting for grid analysis vision to "
                   "be published on topic %s - start vision node" %
                   grid_state_topic)

        # Wait for vision node to start
        cnt = 0.0
        while(not rospy.is_shutdown()):
            rospy.sleep(0.1)
            if self._last_grid is not None:
                break
            if cnt > 5.0:
                print err_msg
                sys.exit(1)
            cnt += 0.1

        self._ai = connect_four.BaxterAI()
        self._manipulate = connect_four.PickPlace(limb)
        self._manipulate.get_locations()

        self._manipulate.move_neutral()
        self._manipulate.move_camera()

        raw_input("Please validate ROI for vision - Press enter when complete")

        self._get_name()

    def _on_grid_state(self, msg):
        # 0 - Unoccupied
        # Red 1
        # Yellow 2
        data = eval(msg.data)
        if self.baxter_color == 1:
            self._baxter_pieces = data['baxter_count']
            self._user_pieces = data['user_count']
        else:
            self._baxter_pieces = data['user_count']
            self._user_pieces = data['baxter_count']
        self._total_pieces = self._baxter_pieces + self._user_pieces
        if self.baxter_color == 2:
            for i in xrange(6):
                for j in xrange(7):
                    # invert colors if baxter is yellow
                    if data['board'][i][j] == 2:
                        self._perceived_grid[i][j] = 1
                    elif data['board'][i][j] == 1:
                        self._perceived_grid[i][j] = 2
                    else:
                        self._perceived_grid[i][j] = 0
        else:
            for i in xrange(6):
                for j in xrange(7):
                    self._perceived_grid[i][j] = data['board'][i][j]
        self._last_grid = self._perceived_grid

    def _get_name(self):
        # Get user name
        self.user = str(raw_input("Please enter your name: "))
        print ("Ready to get beat %s?" % self.user)
        good_input = False
        while not good_input:
            self._depth = int(raw_input("Please enter difficulty "
                                                     "level (1-4): "))
            if self._depth < 1 or self._depth > 4:
                print "Invalid difficulty provided"
            else:
                good_input = True
        good_input = False
        while not good_input:
            self._turn = str(raw_input("Please enter who will go first "
                                        "(baxter, me): "))
            if self._turn == 'baxter':
                self._turn = 1
                good_input = True
            elif self._turn == 'me':
                self._turn = 2
                good_input = True
            else:
                print "Invalid start turn provided"

    def turn(self):
        # 42 possible moves
        if self._round >= 42:
            self.game_over = True
            return

        self.print_board(self.grid)

        if self._turn == 1:
            # Baxter's Move
            print "Baxter's Move"
            self._baxter_cnt += 1
            if self._round == 0:
                move = 3
            else:
                move = self._ai.find_move(self.grid, self._depth, self._round)
            self._robot_cnt += 1
            if self._round < 2 and self._manipulate.read_file == 'y':
                self._manipulate.get_piece()
            self._manipulate.move_neutral()
            self._manipulate.place_piece(move)
            self._manipulate.move_neutral()
            self._manipulate.get_piece()
            self._manipulate.move_camera()

            self.grid = self._perceived_grid
        else:
            # User's Move
            self._user_cnt += 1
            print "User's Move"
            self.user_move()

        # Check for winner
        if self.check_winner():
            self.game_over = True
            self.print_board(self.grid)
            if self.winner == 0:
                self._manipulate.tie()
            if self.winner == self.baxter_color:
                self._manipulate.celebrate()
            if self.winner == self.user_color:
                self._manipulate.disappoint()
            print 'GAME OVER - WINNER FOUND'
            return

        # Switch turns
        if self._turn == 1:
            self._turn = 2
        else:
            self._turn = 1

    def check_winner(self):
        # Check for winner

        # Increment round
        self._round += 1

        # Can't have winner before 7th turn
        if self._round < 7:
            return False

        # Step over grid and check for winners originating at that point
        for i in xrange(6):
            for j in xrange(7):
                if self._ai.horz_count(self.grid, self._turn, i, j, 4):
                    self.winner = self._turn
                    print 'FOUND HORIZONTAL WINNER'
                    return True
                elif self._ai.vert_count(self.grid, self._turn, i, j, 4):
                    self.winner = self._turn
                    print 'FOUND VERTICAL WINNER'
                    return True
                elif self._ai.diag_count(self.grid, self._turn, i, j, 4):
                    self.winner = self._turn
                    print 'FOUND DIAGONAL WINNER'
                    return True
        return False

    def user_move(self):
        cnt = 0
        while True:
            if (self._user_pieces == self._user_cnt and
                self._total_pieces == self._round + 1 and
                self._baxter_pieces == self._baxter_cnt):
                cnt += 1
            else:
                cnt = 0
            if cnt > 20:
                break
            rospy.sleep(0.1)
        # nod to show that you saw user move
        self._manipulate.nod()

        self.grid = self._perceived_grid

    def print_board(self, grid):
        # clear screen and print board
        system('clear')
        for i in xrange(len(grid)):
            print grid[i]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-l', '--limb', dest='limb', choices=['left', 'right'],
                        required=True, help='limb to control')
    parser.add_argument('-c', '--color', dest='color',
                        choices=['red', 'yellow'], default='yellow',
                        required=True, help='baxter pieces color')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('rsdk_connect_four_%s' % (args.limb,))

    cf = ConnectFour(args.limb, args.color)

    while not cf.game_over and not rospy.is_shutdown():
        cf.turn()
    print "THE WINNER IS: "
    if cf.winner == 0:
        print 'TIE'
    if cf.winner == cf.baxter_color:
        print 'Baxter'
    if cf.winner == cf.user_color:
        print cf.user

if __name__ == "__main__":
    main()
