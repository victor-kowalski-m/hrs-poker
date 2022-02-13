#!/usr/bin/env python
"""
Hands 

Name            - Probability  - Combinations - how

Royal Flush     - 1 in 649,737 - 4            - 10, 11, 12, 13, 1 (with same type)
Straight Flush  - 1 in 72,193  - 36           - 4, 5, 6, 7, 8 (5 cards in a row with same type)    
Four of a Kind  - 1 in 4,164   - 624.         - same card in all types
Full House      - 1 in 693     - 3,744.       - a pair plus three of a kind in the same hand
Flush           - 1 in 508     - 5,108.       - 5 cards same type
Straight        - 1 in 253     - 10,200       - 5 cards in numerical order (not same type)
Three of a Kind - 1 in 46      - 54,912       - three of one card and two non-paired cards
Two Pair        - 1 in 20      - 123,552.     - two different pairings or sets of the same card in one hand
One Pair        - 1 in 1.36.   - 1,098,240.   - single pairing of the same card
High Card       - 1 in 0.99    - 1,302,540    - with no matching cards

['H2' 'S2' 'C2' 'D2' 'H3'] - a hand representation

inspired from the tic-tac-toe project
https://gist.github.com/qpwo/c538c6f73727e254fdc7fab81024f6e1

The main script for the game, combining the thinking/planing, trash talking, card and marker detection and the movements of the NAO
Game process will be controlled here through the PokerBard Instance

PokerBoard(tup=(None,) *9, turn=False, winner=None, terminal=False, money_machine=30, money_middle=0, money_opp=30, raised_opp=False, checked_opp=False, raised_ma=False, checked_ma=False, raised_money_opp=0, raised_money_ma=0, folded=None)

"""

from collections import namedtuple
from random import choice
import numpy as np
from numpy.lib.function_base import bartlett
from cards import CardScores
from monte_carlo_tree_search import MCTS, Node
import random
import copy
import rospy
import time
from project.srv import Move
import trash_talk
from project.msg import CardList
import roslaunch
import subprocess
# import atexit
import signal
from naoqi_bridge_msgs.msg import HeadTouch

from func_timeout import func_timeout, FunctionTimedOut

random_use = False

num2words = {5: 'Five', 10: 'Ten', 15: 'Fifteen', 20: 'Twenty',}
words2num = {'Five': 5, 'Ten': 10, 'Fifteen': 15, 'Twenty': 20}
action2key = {'Raise': 'R', 'Fold': 'F', 'Check': 'C', 'All in': 'AI'}
num_list = ["Five", "Ten", "Fifteen", "Twenty"]


### Procedure on exit of the script.

def on_exit(signal_received, frame):
    """
    Function executed on exit of script, goes to rest position 
    and disables stiffness.
    """

    print("\nEXITING!\n")
    trash_talk.on_exit()
    exit(0)

signal.signal(signal.SIGINT, on_exit) # Ctrl + C
signal.signal(signal.SIGTSTP, on_exit) # Ctrl + Z


### Definition of the ROS structure.

def request_move(move):
    """
    Makes a request to the movement service.
    """

    rospy.wait_for_service('move_service')
    print("Move: "+move)
    try:
        move_service_proxy = rospy.ServiceProxy('move_service', Move)
        resp = move_service_proxy(move)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return
    print(resp.status)


def init_nodes():
    """
    Launches required nodes.
    """

    package = "project"
    aruco_execs = [
        "find_aruco",
        "save_aruco.py"
    ]
    execs = aruco_execs + [
        "move_service.py",
        "detect_hand.py",
        "detect_table.py"
    ]
    procs = dict()

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    for e in execs:
        procs[e] = launch.launch(roslaunch.core.Node(package, e, respawn=(True if e not in aruco_execs else False)))

    request_move("st")

    while any(procs[e].is_alive() for e in aruco_execs):
        pass

    request_move("n")

init_nodes()


###  Raise/removal of robot's chips.

# Matrix that keeps track of chips positions (6 blocks of five at the beggining)
chipMat = np.array([
    [1, 1, 0],
    [1, 1, 0],
    [1, 0, 0],
    [1, 0, 0]
])

def print_mat():
    """
    Visual representation of chip matrix state.
    """

    print("\n")
    for i in reversed(range(chipMat.shape[0])):
        for j in reversed(range(chipMat.shape[1])):
            print "o" if chipMat[i, j] else " ", 
        print("\n")
    print("\n")


def remove_chips(pos):
    """
    Removes raised chips from the chip matrix.
    """

    global chipMat

    for i in range(len(chipMat)-1, pos[0]-1, -1):
        chipMat[i,pos[1]] = 0


def add_chips(x):
    """
    Adds an amount x of chips to chipMat.
    """
    global chipMat

    n = x//5
    print("Giving the robot "+str(x)+" chips ("+str(n)+" blocks)")

    # Add desired chips from center to outside
    for j in range(chipMat.shape[1]):
        nz = np.nonzero(chipMat[:, j])[0]
        for i in range ((nz[-1] if len(nz) else -1)+1, chipMat.shape[0]):
            chipMat[i, j] = 1
            n -= 1
            if not n:
                return


def choose_raise(x):
    """
    Chooses which raise movement to execute given an amount x of
    chips to raise.
    """
    global chipMat

    # Keeps pushing until there's no amount left to raise
    while x:
        print(str(x) +" to be raised")

        # Tries to push the max number of blocks possible at once
        max_blocks = x//5 if x//5 <= len(chipMat) else len(chipMat)
        for n_blocks in range(max_blocks, 0, -1):

            # Gets which piles contain the desired number of blocks
            candidates = np.nonzero(np.count_nonzero(chipMat, axis=0)//n_blocks)[0]
            if len(candidates)>0:
                stack = random.choice(candidates) # choose random suitable pile
                height = np.nonzero(chipMat[:, stack])[0][-1] - (n_blocks-1)
                print("Pushing stack "+str(stack)+" height "+str(height)+"")
                request_move("ra "+str(stack)+" "+str(height)) # pushes
                print_mat()
                remove_chips((height, stack))
                print_mat()
                x = x - n_blocks*5
                break


def empty_mat():
    """
    Empties the chip matrix.
    """

    global chipMat

    chipMat = np.zeros((4,3))

### Game strategy.

def get_opp_move():
    """
    listens opponents move and converts it to understandable version for the rest of the programm
    works is for checking if actions were detected right. If raised mones isn't in right spot or even detected, ask again
    Output : action , raised_mon  -- to feed in move function
    """
    
    works = False
    while not works:
        answer = trash_talk.listen()
        if len(answer) == 2:
            try:
                opp_move = 'Raise'
                raised_mon = words2num[answer[1]]
                works = True
            except:
                trash_talk.say("Can you say that again?")
        else:
            if answer in num_list:
                opp_move = 'Raise'
                raised_mon = words2num[answer]
            else:
                opp_move = answer
                raised_mon = 0
            works = True

    action = action2key[opp_move]
    return action, raised_mon

_TTTB = namedtuple("PokerBoard", "tup turn winner terminal money_machine money_middle money_opp raised_opp checked_opp raised_ma checked_ma raised_money_opp raised_money_ma folded")

class PokerBoard(_TTTB, Node):

    def find_children(board):
        """
        finding all random children from that step for rollouts
        Input : board
        Output : moves, consists of list of ALL possible moves from that board state
        """
        tmp_child_counter = 0
        moves = set()

        if (board.money_opp == 0 or board.money_machine == 0 )and board.raised_money_opp == board.raised_money_ma: 
            # if either one of them is all in - open all cards

            moves = board.assign_cards_to_middle_all_combinations(1)

            return moves 

        if (board.checked_opp and board.checked_ma) or (board.raised_opp and board.raised_ma and (board.raised_money_opp == board.raised_money_ma)):
            # both checked or both raised the same amount
            # not yet finished, open the next card
            # after opening one card, set both checks to FALSE
            turn = board.turn
            winner = board.winner
            terminal = board.terminal

            if board.tup[5] == None : 
                m3 = board.assign_cards_to_middle_all_combinations(5)
                return m3
                # board = board.assign_cards_to_middle(3)

            elif board.tup[6] == None : 
                m4 = board.assign_cards_to_middle_all_combinations(6)
                return m4
                #board = board.assign_cards_to_middle(4)

            elif board.tup[6] != None :
                winner, hand_with_score_ma, hand_with_score_opp = _find_winner(board.tup,folded=None)
                terminal = True

                # new
                moves.add(PokerBoard(board.tup, turn, winner, terminal, board.money_machine, board.money_middle, board.money_opp, board.raised_opp, board.checked_opp, board.raised_ma, board.checked_ma, board.raised_money_opp, board.raised_money_ma, board.folded))
                return moves

        if board.terminal:  # If the game is finished then no moves can be made
            return set() 

        
        if board.turn: # turn machine
            if (board.raised_opp and board.raised_ma and (board.raised_money_opp > board.raised_money_ma)) or board.raised_opp and not board.raised_ma:
                # both raised but opp has raised more 
                # can raise
                if board.money_machine >= (board.raised_money_opp - board.raised_money_ma) : 
                    moves.add(board.make_move('R', board.raised_money_opp - board.raised_money_ma))
                else:
                    moves.add(board.make_move('R', board.money_machine))

                # can also fold
                moves.add(board.make_move('F', 0))


            elif not board.raised_opp and (not board.checked_ma or not board.raised_ma): # if opponnent didnt raise and machine didnt checked or raised before
                # can check 
                moves.add(board.make_move('C', 0))
                # can raise if didnt raised before and still has money
                if not board.raised_ma and board.money_machine > 0:

                    if board.money_machine <= board.money_opp :
                        money_intervall = int(board.money_machine/5)

                    else : # if opponnent has less money, no need to raise more than he/she
                        money_intervall = int(board.money_opp/5)

                    for intervall in range(1,money_intervall+1):
                        moves.add(board.make_move('R', 5*intervall))

                
        else: # turn oppponent
            if (board.raised_opp and board.raised_ma and (board.raised_money_ma > board.raised_money_opp)) or board.raised_ma and not board.raised_opp:
                # both raised but opp has raised more 

                # can raise
                if board.money_opp >= board.raised_money_ma - board.raised_money_opp : 
                    moves.add(board.make_move('R', board.raised_money_ma - board.raised_money_opp))
                else:
                    moves.add(board.make_move('R', board.money_opp))

                # can also fold
                moves.add(board.make_move('F', 0))

            elif not board.raised_opp and (not board.checked_opp or not board.raised_opp): # if opponnent didnt raise 
                # can check 
                moves.add(board.make_move('C', 0))
                # can raise if didnt raised before and still has money
                if not board.raised_opp and board.money_opp > 0:

                    if board.money_opp <= board.money_machine :
                        money_intervall = int(board.money_opp/5)

                    else : # if opponnent has less money, no need to raise more than he/she
                        money_intervall = int(board.money_machine/5)

                    for intervall in range(1,money_intervall+1):
                        moves.add(board.make_move('R', 5*intervall))

        return moves

    def find_random_child(board):
        """
        finding A random child from the current board step to use in the simulations
        Input : board
        Output : randomly chosen movement from all possibilities
        """
        if (board.money_opp == 0 or board.money_machine == 0 )and board.raised_money_opp == board.raised_money_ma: 
            # if either one of them is all in - open all cards
            board = board.assign_cards_to_middle(5)
            board = board.assign_cards_to_middle(6)

            winner, hand_with_score_ma, hand_with_score_opp = _find_winner(board.tup, folded=None)
            terminal = True
            turn = board.turn

            board = PokerBoard(board.tup, turn, winner, terminal, board.money_machine, board.money_middle, board.money_opp, board.raised_opp, board.checked_opp, board.raised_ma, board.checked_ma, board.raised_money_opp, board.raised_money_ma, board.folded)
            # print('RANDOM BAOARD : ', board)
            return board

        if (board.checked_opp and board.checked_ma) or (board.raised_opp and board.raised_ma and (board.raised_money_opp == board.raised_money_ma)):
            # both checked or both raised the same amount
            # not yet finished, open the next card
            # after opening one card, set both checks to FALSE
            turn = board.turn
            winner = board.winner
            terminal = board.terminal

            if board.tup[5] == None : 
                board = board.assign_cards_to_middle(5)

            elif board.tup[6] == None : 
                board = board.assign_cards_to_middle(6)

            elif board.tup[6] != None :
                winner, hand_with_score_ma, hand_with_score_opp = _find_winner(board.tup, folded=None)
                terminal = True

            checked_opp = False   
            checked_ma = False   
            raised_opp = False   
            raised_ma = False

            board = PokerBoard(board.tup, turn, winner, terminal, board.money_machine, board.money_middle, board.money_opp, raised_opp, checked_opp, raised_ma, checked_ma, board.raised_money_opp, board.raised_money_ma, board.folded)
            # print('RANDOM BAOARD : ', board)
            return board

        if board.terminal:  # If the game is finished then no moves can be made
            return None

        empty_spots = []
        if board.turn: # turn machine

            if (board.raised_opp and board.raised_ma and (board.raised_money_opp > board.raised_money_ma)) or board.raised_opp and not board.raised_ma:
                # both raised but opp has raised more 

                # can raise
                if board.money_machine >= (board.raised_money_opp - board.raised_money_ma) : 
                    empty_spots.append(['R', board.raised_money_opp - board.raised_money_ma])
                else:
                    empty_spots.append(['R', board.money_machine])

                # can also fold
                empty_spots.append(['F', 0])

            elif not board.raised_opp and (not board.checked_ma or not board.raised_ma): # if opponnent didnt raise and machine didnt checked or raised before
                # can check 
                empty_spots.append(['C', 0])
                # can raise if didnt raised before and still has money

                if not board.raised_ma and board.money_machine > 0:

                    if board.money_machine <= board.money_opp :
                        money_intervall = int(board.money_machine/5)

                    else : # if opponnent has less money, no need to raise more than he/she
                        money_intervall = int(board.money_opp/5)

                    for intervall in range(1,money_intervall+1):
                        empty_spots.append(['R', 5*intervall])

            
        else: # turn oppponent
            if (board.raised_opp and board.raised_ma and (board.raised_money_ma > board.raised_money_opp)) or board.raised_ma and not board.raised_opp:
                # both raised but opp has raised more 

                # can raise
                if board.money_opp >= (board.raised_money_ma - board.raised_money_opp) : 
                    empty_spots.append(['R', board.raised_money_ma - board.raised_money_opp])
                else:
                    empty_spots.append(['R', board.money_opp])

                # can also fold
                empty_spots.append(['F', 0])

            elif not board.raised_opp and (not board.checked_opp or not board.raised_opp): # if opponnent didnt raise 
                # can check 
                empty_spots.append(['C', 0])
                # can raise if didnt raised before and still has money

                if not board.raised_opp and board.money_opp > 0:

                    if board.money_opp <= board.money_machine :
                        money_intervall = int(board.money_opp/5)

                    else : # if opponnent has less money, no need to raise more than he/she
                        money_intervall = int(board.money_machine/5)

                    for intervall in range(1,money_intervall+1):
                        empty_spots.append(['R', 5*intervall])

        random_choice = choice(empty_spots)
        # print('Random CHOICE :\n ', random_choice)

        board = board.make_move(random_choice[0], random_choice[1])
        # print('RANDOM BAOARD : ', board)

        # return board.make_move(random_choice[0], random_choice[1])
        return board

    def reward(board):
        """
        calculate the reward for the terminal state for rollouts
        Input : board
        Output : reward
        """
        reward_to_get = board.money_middle
        if not board.terminal:
            raise RuntimeError("reward called on nonterminal board ")
        if board.winner :
            return -reward_to_get
        else :
            return reward_to_get

        raise RuntimeError("board has unknown winner type")

    def is_terminal(board):
        """
        returns terminal state
        Input : board
        Output : terminal boolen
        """
        return board.terminal

    def assign_cards(board, deck, cards=[]):
        """
        assigning cards to machine in the beginning of the game
        Input : deck, cards
        Output : PokerBoard with cards of the machine
        """
        random_card_list = []
        tup_tmp = board.tup

        if cards == []:
            # if no specific cards are given, assign random cards which are not already in the game
            for index in range(0,2): # give card machines
                random_card = random.choice(deck)
                deck.remove(random_card)

                tup = tup_tmp[:index] + (random_card,) + tup_tmp[index + 1 :]
                tup_tmp = tup
        else: 
            # if specfic cards are given, give these cards to machine
            for index in range(0,2): # give card machines

                tup = tup_tmp[:index] + (cards[index],) + tup_tmp[index + 1 :]
                tup_tmp = tup

        return PokerBoard(tup, board.turn, board.winner, board.terminal, board.money_machine, board.money_middle, board.money_opp, board.raised_opp, board.checked_opp, board.raised_ma, board.checked_ma, board.raised_money_opp, board.raised_money_ma, board.folded)


    def assign_cards_for_estimation(board, deck):
        """
        assin random cards to opponent while 'removing' the machine cards so that opp doesnt know machine cards
        important for rollouts to calculate rewards, since the cards of opponent are not known during simulations
        Input : deck
        Output : PokerBoard with cards of the 'RANDOM' opponent cards
        """
        random_card_list = []
        tup_tmp = board.tup
        tup_new = board.tup

        # remove existing cards from deck for random choice
        deck.remove(tup_tmp[0])
        deck.remove(tup_tmp[1])
        deck.remove(tup_tmp[2])
        deck.remove(tup_tmp[3])
        deck.remove(tup_tmp[4])

        if tup_tmp[5] != None:
            deck.remove(tup_tmp[5])

        if tup_tmp[6] != None:
            deck.remove(tup_tmp[6])

        # choose random card for opp
        random_card = random.choice(deck)
        deck.remove(random_card)

        tup = tup_tmp[:0] + (random_card,) + tup_tmp[0 + 1 :]
        tup_tmp = tup

        random_card = random.choice(deck)
        tup = tup_tmp[:1] + (random_card,) + tup_tmp[1 + 1 :]

        return PokerBoard(tup, board.turn, board.winner, board.terminal, board.money_machine, board.money_middle, board.money_opp, board.raised_opp, board.checked_opp, board.raised_ma, board.checked_ma, board.raised_money_opp, board.raised_money_ma, board.folded)


    def get_opponents_real_cards(board):
        """
        get opponents real cards from camera in the table
        Input : board
        Output : PokerBoard with REAL opponent cards from the table, they should be showed to the camera
        """
        trash_talk.say("show me what you got baby!")

        if not random_use :
            request_move("sh")

        while not rospy.is_shutdown():
            message = rospy.wait_for_message('/table_cards', CardList)
            card_list_middle = message.cards
            print(card_list_middle)
            if len(card_list_middle) == 2 and not any('U' in card for card in card_list_middle):
                break

        # get real opp cards, which the machine doesn't know untill now
        board = board.assign_cards_to_middle(index=7,card=card_list_middle[0])
        board = board.assign_cards_to_middle(index=8,card=card_list_middle[1])

        return board


    def assign_cards_to_middle_all_combinations(board,index):
        """
        For finding "random children" --- random cards should be assigned to middle for rollouts
        Input : board, index(which card shoudl technically be opened)
        Random cards assignment for the desired index in the board
        Output : PokerBoard with RANDOM cardsin the given index position
        """
        moves = set()
        winner = None
        terminal = False

        checked_opp = False   
        checked_ma = False   
        raised_opp = False   
        raised_ma = False

        turn = not board.turn

        card_combos_in = CardScores()
        deck = card_combos_in.build_deck()

        already_exists = []
        already_exists.append(board.tup[0]) #card 1 machine 
        already_exists.append(board.tup[1]) #card 2 machine

        already_exists.append(board.tup[2]) #card 1 middle
        already_exists.append(board.tup[3]) #card 2 middle
        already_exists.append(board.tup[4]) #card 3 middle


        if index == 6 :
            # assigning index 4 -- opening 3rd card in the middle, should check the second card in the middle as well
            already_exists.append(board.tup[5])

        for already_existing_card in already_exists:
            deck.remove(already_existing_card)

        if index == 1 :
            combi = card_combos_in.combinations(deck,2)
            # all 2 card possibilities will be added as possibility for opponents cards

            for combi_possibilities in combi:
                tup = board.tup[:5] + (combi_possibilities[0],) + board.tup[5 + 1 :]
                tup = tup[:6] + (combi_possibilities[1],) + tup[6 + 1 :]

                moves.add(PokerBoard(tup, turn, winner, terminal, board.money_machine, board.money_middle, board.money_opp, raised_opp, checked_opp, raised_ma, checked_ma, board.raised_money_opp, board.raised_money_ma, board.folded))

            return moves

        for random_card in deck :
            tup = board.tup[:index] + (random_card,) + board.tup[index + 1 :]

            if index == 6 : 
                winner, hand_with_score_ma, hand_with_score_opp = _find_winner(tup, folded=None)
                terminal = True

            moves.add(PokerBoard(tup, turn, winner, terminal, board.money_machine, board.money_middle, board.money_opp, raised_opp, checked_opp, raised_ma, checked_ma, board.raised_money_opp, board.raised_money_ma, board.folded))

        return moves

    def assign_cards_to_middle(board,index,card=None):
        """
        For assigning cards to the given index (random or specific cards)
        Input : board, index(which card shoudl technically be opened), index(which position)
        Random cards assignment for the desired index in the board
        Output : PokerBoard with opened card in the given index
        """
        if card is not None :
            # if a card is given, use direclty tha card
            random_card = card

        else:
            # if no card id given, create a random card, whihc is not 'opened' in the middle
            card_combos_in = CardScores()
            deck = card_combos_in.build_deck()
            random_card = random.choice(deck)

            while random_card in board.tup:
                deck.remove(random_card)
                random_card = random.choice(deck)

        # assign new card to board
        tup = board.tup[:index] + (random_card,) + board.tup[index + 1 :]
        turn = not board.turn

        return PokerBoard(tup, turn, board.winner, board.terminal, board.money_machine, board.money_middle, board.money_opp, board.raised_opp, board.checked_opp, board.raised_ma, board.checked_ma, board.raised_money_opp, board.raised_money_ma, board.folded)

    def make_move(board, action, raised_mon,real=False):
        """
        Used both for random rollouts as well as real game
        Makes the moves in the board and returns the new board setting after the move
        Input : board, action(which action is happening R-raise, C-check, F-fold, raised_mon(how much money is raised or even if raised), real(if real game or simulation)
        Output : PokerBoard after the desired action is happened
        """
        if action == 'R': # RAISED

            checked_ma = False
            checked_opp = False
            folded = None

            if board.turn: # turn machine
                raised_ma = True
                raised_money_ma = board.raised_money_ma + raised_mon
                money_machine = board.money_machine - raised_mon
                money_middle = raised_mon + board.money_middle

                # get old values for opp
                raised_opp = board.raised_opp
                raised_money_opp = board.raised_money_opp
                money_opp = board.money_opp
            else:
                raised_opp = True
                raised_money_opp = board.raised_money_opp + raised_mon
                money_opp = board.money_opp - raised_mon 
                money_middle = board.money_middle + raised_mon

                # get old values for machine
                raised_ma = board.raised_ma
                raised_money_ma = board.raised_money_ma
                money_machine = board.money_machine


        elif action == 'C': # CHECKED 

            folded = None
            money_middle = board.money_middle

            # get old values for opp
            raised_money_opp = board.raised_money_opp
            money_opp = board.money_opp

            # get old values for machine
            raised_money_ma = board.raised_money_ma
            money_machine = board.money_machine

            if board.turn: # turn machine
                checked_ma = True
                raised_ma = False
                checked_opp = board.checked_opp
                raised_opp = board.raised_opp
                
         
            else: # turn opp
                checked_opp = True
                raised_opp = False
                checked_ma = board.checked_ma
                raised_ma = board.raised_ma
                         

        elif action == 'F': #  FOLDED
            checked_ma = False
            checked_opp = False
            money_middle = board.money_middle
            # get old values for opp
            raised_opp = board.raised_opp
            raised_money_opp = board.raised_money_opp
            money_opp = board.money_opp
            # get old values for machine
            raised_ma = board.raised_ma
            raised_money_ma = board.raised_money_ma
            money_machine = board.money_machine

            if board.turn:
                folded = 'ma'
            else:
                folded = 'opp'

        turn = not board.turn
        #if ((board.checked_opp and board.checked_ma) or (board.raised_opp and board.raised_ma and (board.raised_money_opp == board.raised_money_ma))) or ((board.money_opp == 0 or board.money_machine == 0 )and board.raised_money_opp == board.raised_money_ma):
        if not real :
            winner, hand_with_score_ma, hand_with_score_opp = _find_winner(board.tup, folded)

        else :
            if folded == 'ma' :
                winner = False
            elif folded == 'opp':
                winner = True
            else :
                winner = None

        is_terminal = (winner is not None) # or not any(v is None for v in tup)

        return PokerBoard(board.tup, turn, winner, is_terminal, money_machine, money_middle, money_opp, raised_opp, checked_opp, raised_ma, checked_ma, raised_money_opp, raised_money_ma, folded)


def opponent_estimation(board,tree): 
    """
    estimate opponents moves and print inside the funtion, no return!
    Input : board, tree
    """
    card_combos = CardScores()
    deck_new = card_combos.build_deck()
    for _ in range(10): # to assign n times random cards to opp for simulation
        deck_new = card_combos.build_deck()
        # create a copy of the board state so that the simulations for opponent can be done
        opponent_guess_board = copy.deepcopy(board)
        # assin random cards to opponent while 'removing' the machine cards so that opp doesnt know machine cards
        opponent_guess_board = opponent_guess_board.assign_cards_for_estimation(deck_new)
        # do simulations for the opponnet
        for _ in range(100):
            # time.sleep(3)
            tree.do_rollout(opponent_guess_board) # Whatever your function that might hang

    # 3 levels are declared for opponent
    # level 3 : pro - takes the best action from simulations
    # level 2 : middle - chooses random from best and second best action from simulation
    # level 1 : noob - chooses random from third and second best action from simulation
    opponent_guess_board = tree.choose_estimate_with_level(opponent_guess_board,level=2)

    if opponent_guess_board.raised_money_opp > board.raised_money_opp:
        print('OPPPONNENT WILL PROBABLY RAISE MONEY BY ', opponent_guess_board.raised_money_opp-board.raised_money_opp)

    elif opponent_guess_board.checked_opp != board.checked_opp:
        print('OPPONNENT WILL PROBABLY CHECK')

    elif opponent_guess_board.folded == 'opp':
        print('OPPONNENT WILL PROBABLY FOLD')


def machine_decision(board,tree): 
    """
    do rollouts for the machine and choose the best action
    Input :board, tree
    Output :board after machine decision
    """
    for _ in range(300):
        tree.do_rollout(board)

    board = tree.choose(board)

    return board


def play_game():
    """
    MAIN PLAY GAME FUNCTION REAL
    all the game structure happens here
    """
    tree = MCTS()
    card_combos = CardScores()
    deck = card_combos.build_deck()

    rospy.init_node('play',anonymous=True)
    # create cards to play with 
    # 2 for machine, 2 for opp, 3 for middle
    # 2598960 combinations in total
    board = new_poker_board()
    request_move("i")

    still_playing = True
    board_exists = False
    first_round = True

    print("Wanna play?")

    while not rospy.is_shutdown():
        data = rospy.wait_for_message("/tactile_touch", HeadTouch)
        if data.state:
            if data.button == 1:
                break

    trash_talk.talk("beginning")

    while still_playing:
        if first_round:
            first_round = False
        else :
            print("ENTERED")
            request_move("i")
        print('BOARD EX : ', board_exists)

        if board_exists :
            trash_talk.say("Another round ?")

            print("Another round ?")

            while not rospy.is_shutdown():
                data = rospy.wait_for_message("/tactile_touch", HeadTouch)
                if data.state:
                    if data.button == 1:
                        break

            if board.winner:
                # machine won last game
                trash_talk.talk("begin second game + won last")
            elif not board.winner:
                # machine lost last
                trash_talk.talk("begin second game + lost last")

            tup=(None,) *9
            turn=False
            winner=None
            terminal=False
            raised_opp=False
            checked_opp=False
            raised_ma=False 
            checked_ma=False
            raised_money_opp=0
            raised_money_ma=0
            folded=None
            money_middle = 0

            board = PokerBoard(tup, turn, winner, terminal, old_board.money_machine, money_middle, old_board.money_opp, raised_opp, checked_opp, raised_ma, checked_ma, raised_money_opp, raised_money_ma, folded)
            if board.money_machine == 0 :
                trash_talk.say("sorry, I don't have any money left. Do you think money grows on trees?")
                print('sorry, machine doesnt have any money left')
                break
            elif board.money_opp == 0:
                trash_talk.say("sorry buddy, but it seems like you don't have any money left")
                print('sorry, you dont have any money left')
                break

        if random_use :
            # ASSIGN RANDOM CARDS TO MACHINE
            board = board.assign_cards(deck)

        else :
            # ALTERNATIVE FROM DETECTION !
            print("buraya")
            while not rospy.is_shutdown():
                print("before wait_for...")
                message = rospy.wait_for_message('/hand_cards', CardList)
                card_list_hand = message.cards
                print(card_list_hand)
                # if len(card_list_hand) >= 2 and not any('U' in card for card in card_list_hand):
                if len(card_list_hand) == 2 and not any('U' in card for card in card_list_hand):
                    break

            board = board.assign_cards_to_middle(index=0,card=card_list_hand[0])
            board = board.assign_cards_to_middle(index=1,card=card_list_hand[1])

        print('BOARD IN THE BEGINNING : ', board)

        turn_iteration = 1

        while True:
            action_check = False
            if not board.turn : # if turn opponnent 

                while not action_check:   
                    trash_talk.say("your turn")
                    action, raised_mon = get_opp_move()
                    
                    if turn_iteration == 1 and action != 'R' :
                        print('MUST RAISE IN THE FIRST ROUND')
                        trash_talk.say('YOU MUST RAISE IN THE FIRST ROUND')

                    elif action == 'F':
                        raised_mon = 0
                        action_check = True

                        if not random_use :
                            request_move("f")
                        # need somehow to get all the money from middle !!! - for the reward system can be important
                        break
                
                    elif action == 'R':
                        action_check_2 = False
                        while not action_check_2:
                            if (int(raised_mon)+board.raised_money_opp) < board.raised_money_ma :
                                print('you need to raise at least as much as me')
                                trash_talk.say('you need to raise at least as much as me, what is your move ?')
                                action_check_2 = True

                            elif (board.raised_ma and (int(raised_mon)+board.raised_money_opp > board.money_machine + board.raised_money_ma)) or (not board.raised_ma and (int(raised_mon) > board.money_machine)):
                            #elif (board.raised_money_ma < board.raised_money_opp + int(raised_mon)) and int(raised_mon) > board.money_machine:
                                print('cant raise more money than your opponnent has, doesnt make sense :D' )
                                trash_talk.say('maybe you want to think about this move')
                                action_check_2 = True
                            else:
                                print('YOU RAISED BY ', raised_mon)
                                action_check_2 = True
                                action_check = True

                    elif action == 'AI':
                        action = 'R'

                        if (board.money_opp + board.raised_money_opp) > (board.money_machine + board.raised_money_ma):
                            raised_mon = (board.money_machine + board.raised_money_ma)-board.raised_money_opp
                        else:
                            raised_mon = board.money_opp
                        action_check = True

                    elif action == 'C':
                        if board.raised_ma:
                            print('CANT CHECK, MONEY RISED!')
                            trash_talk.say("Can't you see, I raised money? Maybe you should have your eyes check!")
                        else :
                            raised_mon = 0
                            action_check = True
                            print('YOU CHECKED')

                board = board.make_move(action, int(raised_mon),real=True)


                if board.terminal and (board.folded == None):
                    winner = None
                    terminal = False
                    board = PokerBoard(board.tup, board.turn, winner, terminal, board.money_machine, board.money_middle, board.money_opp, board.raised_opp, board.checked_opp, board.raised_ma, board.checked_ma, board.raised_money_opp, board.raised_money_ma, board.folded)


            print('\n')

            if turn_iteration == 1 :
                # first round
                # machine can only raise for now

                board = board.make_move('R', int(raised_mon),real=True)

                trash_talk.say("I also raise by " + num2words[int(raised_mon)])
                if not random_use :
                    choose_raise(int(raised_mon))
                    ### must be changed!! 
                print('MACHINE ALSO RAISES  ')
                print("I also raise by " + num2words[int(raised_mon)])
            
                # new cards in the middle should be opened

                # FROM DETECTION
                if not random_use :
                    while not rospy.is_shutdown():
                        message = rospy.wait_for_message('/table_cards', CardList)
                        card_list_middle = message.cards
                        print(card_list_middle)
                        # should detetct 3 cards at the same time from middle
                        if len(card_list_middle) == 3 and not any('U' in card for card in card_list_middle):
                            break

                    board = board.assign_cards_to_middle(index=2,card=card_list_middle[0])
                    board = board.assign_cards_to_middle(index=3,card=card_list_middle[1])
                    board = board.assign_cards_to_middle(index=4,card=card_list_middle[2])

                else :
                    # ASSIGNING RANDOM CARDS
                    board = board.assign_cards_to_middle(2)
                    board = board.assign_cards_to_middle(3)
                    board = board.assign_cards_to_middle(4)

                turn = not board.turn

                checked_opp = False   
                checked_ma = False   
                raised_opp = False   
                raised_ma = False

                board = PokerBoard(board.tup, turn, board.winner, board.terminal, board.money_machine, board.money_middle, board.money_opp, raised_opp, checked_opp, raised_ma, checked_ma, board.raised_money_opp, board.raised_money_ma, board.folded)
                print('NEW 3 CARDS ARE OPENED IN THE MIDDLE')
                print('CURRENT CARDS IN THE MIDDLE : ', board.tup)
                turn_iteration += 1
                board_exists = True

                board = check_if_open_new_card(board)

                terminal_state,board = check_board_terminal(board)
                if terminal_state :
                    old_board = board
                    break
                
            else : # after the first turn (after opening the 3 cards in the middle)

                print('BOARD AFTER HUMAN MOVE : \n', board)
                print('\n')

                # check if new cards should be opened
                board = check_if_open_new_card(board)
                board_before_move = board

                terminal_state,board = check_board_terminal(board)
                if terminal_state :
                    old_board = board
                    break
                
                # run rollouts for the machine with time limit
                try:
                    board = func_timeout(4, machine_decision, args=(board, tree))
                except FunctionTimedOut:
                    # if time exceeds
                    print( "Machine decicion time out, skip it dont wait it")
                    turn = not board.turn
                    if board.checked_opp :
                        trash_talk.say("Check.")
                        checked_ma = True
                        # just check as default
                        board = PokerBoard(board.tup, turn, board.winner, board.terminal, board.money_machine, board.money_middle, board.money_opp, board.raised_opp, board.checked_opp, board.raised_ma, checked_ma, board.raised_money_opp, board.raised_money_ma, board.folded)
                    elif board.raised_opp and not board.raised_ma:
                        raised_ma = True
                        # if raised money opponent, raise as much for default
                        raised_money_ma = board.raised_money_opp
                        trash_talk.say("I raise by " + str(raised_money_ma))
                        board = PokerBoard(board.tup, turn, board.winner, board.terminal, board.money_machine, board.money_middle, board.money_opp, board.raised_opp, board.checked_opp, raised_ma, board.checked_ma, board.raised_money_opp, raised_money_ma, board.folded)


                # dont wanna end the game with rollout moves (comes from simulation)
                if board.terminal and (board.folded == None):
                    winner = None
                    terminal = False
                    board = PokerBoard(board.tup, board.turn, winner, terminal, board.money_machine, board.money_middle, board.money_opp, board.raised_opp, board.checked_opp, board.raised_ma, board.checked_ma, board.raised_money_opp, board.raised_money_ma, board.folded)


                if board.raised_money_ma > board_before_move.raised_money_ma:
                    if board.money_machine == 0:
                        print('MACHINE ALL IN')

                        trash_talk.talk("me all in")
                        # machine all in
                        if not random_use :
                            furthest_stack = 0
                            for x in chipMat[0][1:]:
                                if x:
                                    furthest_stack += 1
                            request_move("ai "+str(furthest_stack))
                            empty_mat()
                    else:
                        amount = board.raised_money_ma-board_before_move.raised_money_ma
                        print('MACHINE RAISED MONEY BY ', amount)
                        # machine has raised

                        trash_talk.say("I raise by " + num2words[amount])
                        if not random_use :
                            choose_raise(amount)
                        if random.randint(0,1):
                            trash_talk.talk("random")

                elif board.checked_ma != board_before_move.checked_ma:
                    print('MACHINE CHECKED')
                    # machine has checked -- so that both checkeds are set to false
                    trash_talk.say("Check.")

                    if not random_use :
                        request_move("c")
            
                elif board.folded == 'ma':
                    print('MACHINE FOLDED')

                    trash_talk.say("I fold.")

                    # machine has folded
                    if not random_use :
                        request_move("f")

                print('\n')
                print('BOARD AFTER MACHINE DECISION : \n', board)
                print('\n')

                # check if new cards should be opened
                board = check_if_open_new_card(board)

                terminal_state,board = check_board_terminal(board)
                if terminal_state :
                    old_board = board
                    break

                ######  estimate opponents moves! ######
                try:
                    doitReturnValue = func_timeout(3, opponent_estimation, args=(board, tree))
                except FunctionTimedOut:
                    print ( "Oppinent estimation time out, skip it dont wait it")
                #####  opponent estimation done! ######

def check_board_terminal(board):
    """
    checking if board reached the terminal state
    Input : board current state
    Output : terminal_state (boolean),board
    """
    if board.terminal:
        # if game finished, calculate the new money distribution between players
        print('TERMINAL')
        money_machine = board.money_machine
        money_opp = board.money_opp
        money_middle = board.money_middle

        if board.winner : # machine won
            print_mat()
            add_chips(board.money_middle)
            print_mat()
            # gets the money in the middle
            money_machine = board.money_middle + board.money_machine
        elif board.winner == False:
            # opp gets the money in the middle
            money_opp = board.money_middle + board.money_opp

        # talk win or loose
        trash_talk.talk('won' if board.winner else 'lost')

        money_middle = 0

        board = PokerBoard(board.tup, board.turn, board.winner, board.terminal, money_machine, money_middle, money_opp, board.raised_opp, board.checked_opp, board.raised_ma, board.checked_ma, board.raised_money_opp, board.raised_money_ma, board.folded)
        old_board = board
        return True, old_board
    else:
        return False, board


def check_if_open_new_card(board):
    """
    checking if a new card should be opened in the middle
    + if game is finished -- if yes checking the winner
    Input : board current state
    Output : PokerBoard with opened card in the given index
    """

    if board.terminal:
        print('GAME OVER')

        if board.folded == 'opp':
            winner = True
        elif board.folded == 'ma':
            winner = False
        else : 

            board = board.get_opponents_real_cards()
            winner, hand_with_score_ma, hand_with_score_opp, c_h_best, c_o_best  = _find_winner_real(board.tup, folded=None)

            trash_talk.say("I have a hand of " + str(hand_with_score_ma))
            trash_talk.say("You have a hand of " + str(hand_with_score_opp))

            if winner: 
                trash_talk.say("I won")
            else:
                trash_talk.say("you won")

        print('winner : ', 'MACHINE' if winner else 'OPPONNENT')

    if (board.money_opp == 0 or board.money_machine == 0 )and board.raised_money_opp == board.raised_money_ma: 
        # if either one of them is all in - open all cards
        print('ALL IN, OPENING ALL THE CARDS IN THE MIDDLE')
        print('\n')

        # FROM DETECTION
        if not random_use :
            while not rospy.is_shutdown():
                message = rospy.wait_for_message('/table_cards', CardList)
                card_list_middle = message.cards
                print(card_list_middle)
                if len(card_list_middle) == 5 and not any('U' in card for card in card_list_middle):
                    break

            board = board.assign_cards_to_middle(index=2,card=card_list_middle[0])
            board = board.assign_cards_to_middle(index=3,card=card_list_middle[1])
            board = board.assign_cards_to_middle(index=4,card=card_list_middle[2])
            board = board.assign_cards_to_middle(index=5,card=card_list_middle[3])
            board = board.assign_cards_to_middle(index=6,card=card_list_middle[4])
        else:
            # RANDOM ASSIGNMENT
            board = board.assign_cards_to_middle(5)
            board = board.assign_cards_to_middle(6)

        print('CARDS IN THE MIDDLE')
        print(board.tup)
        
        board = board.get_opponents_real_cards()

        winner, hand_with_score_ma, hand_with_score_opp, c_h_best, c_o_best = _find_winner_real(board.tup, folded=None)

        trash_talk.say("I have a hand of " + str(hand_with_score_ma))
        trash_talk.say("You have a hand of " + str(hand_with_score_opp))

        if winner: 
            trash_talk.say("I won")
        else:
            trash_talk.say("you won")

        if winner is not None :
            print('winner : ', 'MACHINE' if winner else 'OPPONNENT')

            if board.folded is None :

                print('machine : ' + hand_with_score_ma + ' , ' + str(c_h_best))
                print('opponnent : ' + hand_with_score_opp + ' , ' + str(c_o_best))

        terminal = True
        turn = not board.turn
        money_machine = board.money_machine
        money_opp = board.money_opp
        money_middle = board.money_middle

        board = PokerBoard(board.tup, turn, winner, terminal, money_machine, money_middle, money_opp, board.raised_opp, board.checked_opp, board.raised_ma, board.checked_ma, board.raised_money_opp, board.raised_money_ma, board.folded)



    elif (board.checked_opp and board.checked_ma) or (board.raised_opp and board.raised_ma and (board.raised_money_opp == board.raised_money_ma)):
        # both checked or both raised the same amount
        # not yet finished, open the next card
        # after opening one card, set both checks to FALSE

        winner = board.winner
        terminal = board.terminal
        turn = board.turn
        money_machine = board.money_machine
        money_opp = board.money_opp
        money_middle = board.money_middle

        if board.tup[5] == None : 
            if not random_use : 
                # FROM DETECTION
                while not rospy.is_shutdown():
                    message = rospy.wait_for_message('/table_cards', CardList)
                    card_list_middle = message.cards
                    print(card_list_middle)
                    if len(card_list_middle) == 4 and not any('U' in card for card in card_list_middle):
                        break

                board = board.assign_cards_to_middle(index=2,card=card_list_middle[0])
                board = board.assign_cards_to_middle(index=3,card=card_list_middle[1])
                board = board.assign_cards_to_middle(index=4,card=card_list_middle[2])
                board = board.assign_cards_to_middle(index=5,card=card_list_middle[3])
            else :
                board = board.assign_cards_to_middle(5)  ### RANDOM

        elif board.tup[6] == None : 
            # get the last card in the middle
            if not random_use :
                while not rospy.is_shutdown():
                    message = rospy.wait_for_message('/table_cards', CardList)
                    card_list_middle = message.cards
                    print(card_list_middle)
                    if len(card_list_middle) == 5 and not any('U' in card for card in card_list_middle):
                        break

                board = board.assign_cards_to_middle(index=2,card=card_list_middle[0])
                board = board.assign_cards_to_middle(index=3,card=card_list_middle[1])
                board = board.assign_cards_to_middle(index=4,card=card_list_middle[2])
                board = board.assign_cards_to_middle(index=5,card=card_list_middle[3])
                board = board.assign_cards_to_middle(index=6,card=card_list_middle[4])

            else:
                board = board.assign_cards_to_middle(6) ### RANDOM

        elif board.tup[6] != None :
            # all cards are already opened in the middle, ask opponents cards to get winner 
            board = board.get_opponents_real_cards()

            winner, hand_with_score_ma, hand_with_score_opp, c_h_best, c_o_best = _find_winner_real(board.tup,folded=None)

            trash_talk.say("I have a hand of " + str(hand_with_score_ma))
            trash_talk.say("You have a hand of " + str(hand_with_score_opp))

            if winner: 
                trash_talk.say("I won")
            else:
                trash_talk.say("you won")

            terminal = True        

        checked_opp = False   
        checked_ma = False   
        raised_opp = False   
        raised_ma = False

        board = PokerBoard(board.tup, turn, winner, terminal, money_machine, money_middle, money_opp, raised_opp, checked_opp, raised_ma, checked_ma, board.raised_money_opp, board.raised_money_ma, board.folded)
        
        print('\nOPENED NEW CARD SINCE BOTH PLAYERS RAISED SAME AMOUNT OR BOTH CHECKED IN THE ROUND')
        print('NEW CARDS : ', board.tup)
        print('\nNEW ROUND')
        print('BOARD SITUATION : ', board)


        if winner is not None :
            print('winner : ', 'MACHINE' if winner else 'OPPONNENT')

            if board.folded is None :

                print('machine : ' + hand_with_score_ma + ' , ' + str(c_h_best))
                print('opponnent : ' + hand_with_score_opp + ' , ' + str(c_o_best))

    return board

def _find_winner(tup, folded):
    """
    to find winner in use of the random rollout
    generates oppponnents cards randomly!!
    Input : tup(cards in the PokerBoard), folded(if some player folded)
    Output : winner(True/False --  cant be None), hand_with_score_ma (score of the machine hand), hand_with_score_opp (score of the opp hand)
    """

    hand_with_score_ma = None
    hand_with_score_opp = None

    if folded == None :
        card_combos = CardScores()

        if tup[6] == None : # if the 3rd card in the middle IS NOT OPENED YET
            winner = None

        else:
            machine_hand = []
            opp_hand = []

            # for random opponnent cards
            deck = card_combos.build_deck()
            for i in range(0,7):
                deck.remove(tup[i])
            for i in range (0,2):
                random_card = random.choice(deck)
                opp_hand.append(random_card)
            #############################

            opp_hand.append(tup[2])
            opp_hand.append(tup[3])
            opp_hand.append(tup[4])
            opp_hand.append(tup[5])
            opp_hand.append(tup[6])

            for i in range (0,7):
                machine_hand.append(tup[i])

            combi_hand = card_combos.combinations(machine_hand,5)
            combi_opp = card_combos.combinations(opp_hand,5)

            # calculate scores machine hand for all combination possibilities
            score_machine = 0
            hand_with_score_ma = None
            for c_h in combi_hand:
                # get the scores for all possible combinations and compare to find best
                score_machine_tmp, hand_with_score_ma_tmp = card_combos.score_hand(c_h)

                if score_machine_tmp*1000 > score_machine*1000:
                    score_machine = score_machine_tmp
                    hand_with_score_ma = hand_with_score_ma_tmp

            # calculate scores opponent hand for all combination possibilities
            score_opp = 0
            hand_with_score_opp = None
            for c_o in combi_opp:
                # get the scores for all possible combinations and compare to find best
                score_opp_tmp, hand_with_score_opp_tmp = card_combos.score_hand(c_o)

                if score_opp_tmp*1000 > score_opp*1000:
                    score_opp = score_opp_tmp
                    hand_with_score_opp = hand_with_score_opp_tmp

            if score_machine*1000 >= score_opp*1000: # for comma position 
                winner = True
            else:
                winner = False

    elif folded == 'opp':
        winner = True
    elif folded == 'ma':
        winner = False
    else :
        print('should never arrive here')

    return winner, hand_with_score_ma, hand_with_score_opp

def _find_winner_real(tup, folded):
    """
    finding winner for the real game, opponents real cards should already been asked
    Input : tup(cards in the PokerBoard), folded(if some player folded)
    Output : winner(True/False --  cant be None), hand_with_score_ma (score of the machine hand), hand_with_score_opp (score of the opp hand), (best HAND combinations) : c_h_best , c_o_best
    """

    hand_with_score_ma = None
    hand_with_score_opp = None
    c_o_best = ''
    c_h_best = ''

    if folded == None :
        card_combos = CardScores()

        if tup[6] == None : # if the 3rd card in the middle IS NOT OPENED YET
            winner = None

        else:
            machine_hand = []
            opp_hand = []

            for i in range (0,7):
                machine_hand.append(tup[i])
                opp_hand.append(tup[i+2]) # get the real values from the tuple
 
            # from 7 cards build the 5 card combination of each hand to calculate scores
            combi_hand = card_combos.combinations(machine_hand,5)
            combi_opp = card_combos.combinations(opp_hand,5)

            # calculate scores machine hand for all combination possibilities
            score_machine = 0
            for c_h in combi_hand:
                # get the scores for all possible combinations and compare to find best
                score_machine_tmp, hand_with_score_ma_tmp = card_combos.score_hand(c_h)

                if score_machine_tmp*1000 > score_machine*1000:
                    score_machine = score_machine_tmp
                    hand_with_score_ma = hand_with_score_ma_tmp
                    c_h_best = c_h # get only the best

            # calculate scores opponent hand for all combination possibilities
            score_opp = 0
            for c_o in combi_opp:
                # get the scores for all possible combinations and compare to find best
                score_opp_tmp, hand_with_score_opp_tmp = card_combos.score_hand(c_o)

                if score_opp_tmp*1000 > score_opp*1000:
                    score_opp = score_opp_tmp
                    hand_with_score_opp = hand_with_score_opp_tmp
                    c_o_best = c_o # get only the best

            # find the winner with comparing the best scores of both hands
            if score_machine*1000 >= score_opp*1000: # for comma positions 
                winner = True
            else:
                winner = False

    # check if someone folded
    elif folded == 'opp':
        winner = True

    elif folded == 'ma':
        winner = False
    else :
        print('should never arrive here')

    return winner, hand_with_score_ma, hand_with_score_opp, c_h_best, c_o_best

def new_poker_board():
    """
    initialize NEW PokerBoard at the initial state    
    Output : PokerBoard in init state
    """
    return PokerBoard(tup=(None,) *9, turn=False, winner=None, terminal=False, money_machine=30, money_middle=0, money_opp=30, raised_opp=False, checked_opp=False, raised_ma=False, checked_ma=False, raised_money_opp=0, raised_money_ma=0, folded=None)


if __name__ == "__main__":
    play_game()

