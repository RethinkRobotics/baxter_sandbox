#!/usr/bin/python

# Four-In-A-Row (a Connect Four clone)
# By Al Sweigart al@inventwithpython.com
# http://inventwithpython.com/pygame
# Released under a "Simplified BSD" license

import argparse
import copy
import random
import pygame
import sys

from os import system
from pygame.locals import *

import rospy

import rospkg

import connect_four


class PyFour(object):
    def __init__(self, depth):

        self._depth = depth

        self._img_path = (rospkg.RosPack().get_path('connect_four')
                          + '/share/')

        pygame.init()
        self._fps_clock = pygame.time.Clock()
        self._display_surf = pygame.display.set_mode((640, 480))
        pygame.display.set_caption('Four in a Row')

        self._red_pile = pygame.Rect(
                                     int(50 / 2),
                                     480 - int(3 * 50 / 2),
                                     50,
                                     50
                         )
        self._black_pile = pygame.Rect(640 - int(3 * 50 / 2),
                                       480 - int(3 * 50 / 2),
                                       50,
                                       50
                           )
        self._red_img = pygame.image.load(self._img_path + '4row_red.png')
        self._red_img = pygame.transform.smoothscale(self._red_img, (50, 50))
        self._black_img = pygame.image.load(self._img_path + '4row_black.png')
        self._black_img = pygame.transform.smoothscale(self._black_img,
                                                       (50, 50))
        self._board_img = pygame.image.load(self._img_path + '4row_board.png')
        self._board_img = pygame.transform.smoothscale(self._board_img,
                                                       (50, 50))

        self._human_win = pygame.image.load(self._img_path +
                                            '4row_humanwinner.png')
        self._comp_win = pygame.image.load(self._img_path +
                                           '4row_computerwinner.png')
        self._tie = pygame.image.load(self._img_path + '4row_tie.png')
        self._winner_rect = self._human_win.get_rect()
        self._winner_rect.center = (int(640 / 2), int(480 / 2))

        self._arrow_img = pygame.image.load(self._img_path + '4row_arrow.png')
        self._arrow_rect = self._arrow_img.get_rect()
        self._arrow_rect.left = self._red_pile.right + 10
        self._arrow_rect.centery = self._red_pile.centery

        self.grid = [[0 for _i in range(6)] for _j in range(7)]
        self.rot_grid = [[0 for _i in range(7)] for _j in range(6)]

        self._ai = connect_four.BaxterAI()

        self.isFirstGame = True

    def runGame(self, isFirstGame):
        # Randomly choose who goes first.
        if random.randint(0, 1) == 0:
            turn = 'computer'
        else:
            turn = 'human'
        showHelp = True

        round = 0
        while True:  # main game loop
            for idx, col in enumerate(self.grid):
                for row in xrange(6):
                    self.rot_grid[row][idx] = col[row]
            if turn == 'human':
                # Human player's turn.
                self.getHumanMove(self.grid, showHelp)
                if showHelp:
                    # turn off help arrow after the first move
                    showHelp = False
                if self.isWinner(self.grid, 2):
                    print "HUMAN WINS"
                    winnerImg = self._human_win
                    break
                turn = 'computer'  # switch to other player's turn
            else:
                # Computer player's turn.
                column = self._ai.find_move(self.rot_grid, self._depth, round)
                self.animateComputerMoving(self.grid, column)
                self.makeMove(self.grid, 1, column)
                if self.isWinner(self.grid, 1):
                    print "ROBOT WINS"
                    winnerImg = self._comp_win
                    break
                turn = 'human'  # switch to other player's turn

            if self.isBoardFull(self.grid):
                # A completely filled board means it's a tie.
                print "TIE"
                winnerImg = self._tie
                break

            round += 1

    def makeMove(self, board, player, column):
        lowest = self.lowestSpace(board, column)
        if lowest != -1:
            board[column][lowest] = player

    def drawBoard(self, board, extraToken=0):
        self._display_surf.fill((0, 50, 255))

        # draw tokens
        spaceRect = pygame.Rect(0, 0, 50, 50)
        for x in range(7):
            for y in range(6):
                spaceRect.topleft = (145 + (x * 50), 90 + (y * 50))
                if board[x][y] == 2:
                    self._display_surf.blit(self._red_img, spaceRect)
                elif board[x][y] == 1:
                    self._display_surf.blit(self._black_img, spaceRect)

        # draw the extra token
        if extraToken != 0:
            if extraToken['color'] == 2:
                self._display_surf.blit(
                    self._red_img,
                    (extraToken['x'], extraToken['y'], 50, 50)
                )
            elif extraToken['color'] == 1:
                self._display_surf.blit(
                    self._black_img,
                    (extraToken['x'], extraToken['y'], 50, 50)
                )

        # draw board over the tokens
        for x in range(7):
            for y in range(6):
                spaceRect.topleft = (145 + (x * 50), 90 + (y * 50))
                self._display_surf.blit(self._board_img, spaceRect)

        # draw the 2 and 1 tokens off to the side
        self._display_surf.blit(self._red_img, self._red_pile)
        self._display_surf.blit(self._black_img, self._black_pile)

    def getHumanMove(self, board, isFirstMove):
        draggingToken = False
        tokenx, tokeny = 0, 0
        while True:
            for event in pygame.event.get():  # event handling loop
                if event.type == QUIT:
                    pygame.quit()
                    sys.exit()
                elif (event.type == MOUSEBUTTONDOWN and not draggingToken and
                      self._red_pile.collidepoint(event.pos)):
                    # start of dragging on 2 token pile.
                    draggingToken = True
                    tokenx, tokeny = event.pos
                elif event.type == MOUSEMOTION and draggingToken:
                    # update the position of the 2 token being dragged
                    tokenx, tokeny = event.pos
                elif event.type == MOUSEBUTTONUP and draggingToken:
                    # let go of the token being dragged
                    if tokeny < 90 and tokenx > 145 and tokenx < (640 - 145):
                        # let go at the top of the screen.
                        column = int((tokenx - 145) / 50)
                        if self.isValidMove(board, column):
                            self.animateDroppingToken(board, column, 2)
                            board[column][self.lowestSpace(board, column)] = 2
                            self.drawBoard(board)
                            pygame.display.update()
                            return
                    tokenx, tokeny = 0, 0
                    draggingToken = False
            if tokenx != 0 and tokeny != 0:
                self.drawBoard(
                    board,
                    {'x': tokenx - int(50 / 2),
                     'y': tokeny - int(50 / 2),
                     'color': 2}
                )
            else:
                self.drawBoard(board)

            if isFirstMove:
                # Show the help arrow for the player's first move.
                self._display_surf.blit(self._arrow_img, self._arrow_rect)

            pygame.display.update()
            self._fps_clock.tick()

    def animateDroppingToken(self, board, column, color):
        x = 145 + column * 50
        y = 90 - 50
        dropSpeed = 1.0

        lowestSpace = self.lowestSpace(board, column)

        while True:
            y += int(dropSpeed)
            dropSpeed += 0.5
            if int((y - 90) / 50) >= lowestSpace:
                return
            self.drawBoard(board, {'x': x, 'y': y, 'color': color})
            pygame.display.update()
            self._fps_clock.tick()

    def animateComputerMoving(self, board, column):
        x = self._black_pile.left
        y = self._black_pile.top
        speed = 1.0
        # moving the 1 tile up
        while y > (90 - 50):
            y -= int(speed)
            speed += 0.5
            self.drawBoard(board, {'x': x, 'y': y, 'color': 1})
            pygame.display.update()
            self._fps_clock.tick()
        # moving the 1 tile over
        y = 90 - 50
        speed = 1.0
        while x > (145 + column * 50):
            x -= int(speed)
            speed += 0.5
            self.drawBoard(board, {'x': x, 'y': y, 'color': 1})
            pygame.display.update()
            self._fps_clock.tick()
        # dropping the 1 tile
        self.animateDroppingToken(board, column, 1)

    def getComputerMove(self, board):
        potentialMoves = self.getPotentialMoves(board, 1, 2)
        # get the best fitness from the potential moves
        bestMoveFitness = -1
        for i in range(7):
            if (potentialMoves[i] > bestMoveFitness and
                self.isValidMove(board, i)):
                bestMoveFitness = potentialMoves[i]
        # find all potential moves that have this best fitness
        bestMoves = []
        for i in range(len(potentialMoves)):
            if (potentialMoves[i] == bestMoveFitness and
                self.isValidMove(board, i)):
                bestMoves.append(i)
        return random.choice(bestMoves)

    def getPotentialMoves(self, board, tile, lookAhead):
        if lookAhead == 0 or self.isBoardFull(board):
            return [0] * 7

        if tile == 2:
            enemyTile = 1
        else:
            enemyTile = 2

        # Figure out the best move to make.
        potentialMoves = [0] * 7
        for firstMove in range(7):
            dupeBoard = copy.deepcopy(board)
            if not self.isValidMove(dupeBoard, firstMove):
                continue
            self.makeMove(dupeBoard, tile, firstMove)
            if self.isWinner(dupeBoard, tile):
                # a winning move automatically gets a perfect fitness
                potentialMoves[firstMove] = 1
                break  # don't bother calculating other moves
            else:
                # do other player's counter moves and determine best one
                if self.isBoardFull(dupeBoard):
                    potentialMoves[firstMove] = 0
                else:
                    for counterMove in range(7):
                        dupeBoard2 = copy.deepcopy(dupeBoard)
                        if not self.isValidMove(dupeBoard2, counterMove):
                            continue
                        self.makeMove(dupeBoard2, enemyTile, counterMove)
                        if self.isWinner(dupeBoard2, enemyTile):
                            # a losing move automatically gets the worst fit
                            potentialMoves[firstMove] = -1
                            break
                        else:
                            # do the recursive call to self.getPotentialMoves()
                            results = self.getPotentialMoves(dupeBoard2, tile,
                                                             lookAhead - 1)
                            potentialMoves[firstMove] += (sum(results) / 7) / 7
        return potentialMoves

    def lowestSpace(self, board, column):
        # Return the row number of the lowest empty row in the given column.
        for y in range(6 - 1, -1, -1):
            if board[column][y] == 0:
                return y
        return -1

    def isValidMove(self, board, column):
        # Returns True if there is an empty space in the given column.
        # Otherwise returns False.
        if column < 0 or column >= (7) or board[column][0] != 0:
            return False
        return True

    def isBoardFull(self, board):
        # Returns True if there are no empty spaces anywhere on the board.
        for x in range(7):
            for y in range(6):
                if board[x][y] == 0:
                    return False
        return True

    def isWinner(self, board, tile):
        # check horizontal spaces
        for x in range(7 - 3):
            for y in range(6):
                if (board[x][y] == tile and board[x + 1][y] == tile and
                    board[x + 2][y] == tile and board[x + 3][y] == tile):
                    return True
        # check vertical spaces
        for x in range(7):
            for y in range(6 - 3):
                if (board[x][y] == tile and board[x][y + 1] == tile and
                    board[x][y + 2] == tile and board[x][y + 3] == tile):
                    return True
        # check / diagonal spaces
        for x in range(7 - 3):
            for y in range(3, 6):
                if (board[x][y] == tile and
                    board[x + 1][y - 1] == tile and
                    board[x + 2][y - 2] == tile and
                    board[x + 3][y - 3] == tile):
                    return True
        # check \ diagonal spaces
        for x in range(7 - 3):
            for y in range(6 - 3):
                if (board[x][y] == tile and
                    board[x + 1][y + 1] == tile and
                    board[x + 2][y + 2] == tile and
                    board[x + 3][y + 3] == tile):
                    return True
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--depth', dest='depth', type=int,
                        choices=[1, 2, 3, 4], required=True,
                        help='skill level (depth of search)')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('test_connect_four_ai')

    pf = PyFour(args.depth)
    while not rospy.is_shutdown() and pf.isFirstGame:
        pf.runGame(pf.isFirstGame)
        pf.isFirstGame = False

if __name__ == '__main__':
    main()
