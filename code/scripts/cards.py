"""
calculating the scores of hands in the poker game
inspired from the git project 
https://gist.github.com/dirusali/bbd2039c7ce3b3a904606c0d60bb6a3b#file-score_hands-py
https://gist.github.com/dirusali/e1184ddec664a96bdcfe36377155c19b#file-hand_types-py

"""
import numpy as np
import itertools


class CardScores:

    def __init__(self):
        return

    def build_deck(self):
        """
        build deck of 52 cards     
        Input : -
        Output : deck
        """
        numbers=list(range(2,15))
        suits = ['H','S','C','D']
        deck = []
        for i in numbers:
            for s in suits:
                card = s+str(i)
                deck.append(card)
        return deck

    def combinations(self, arr, n):
        """
        build card combinations in given size   
        Input : n(number of cards to have in the combination), arr(where you wanna build the combinations)
        Output : combinations 
        """
        arr = np.asarray(arr)
        t = np.dtype([('', arr.dtype)]*n)
        result = np.fromiter(itertools.combinations(arr, n), t)
        return result.view(arr.dtype).reshape(-1, n)

    ############# checking the possible combinations to find in an hand with corresponding score #############
    # returning scores

    def check_four_of_a_kind(self, hand,letters,numbers,rnum,rlet):
        for i in numbers:
                if numbers.count(i) == 4:
                    four = i
                elif numbers.count(i) == 1:
                    card = i
        score = 105 + four + card/100
        return score

    def check_full_house(self, hand,letters,numbers,rnum,rlet):
        for i in numbers:
            if numbers.count(i) == 3:
                full = i
            elif numbers.count(i) == 2:
                p = i
        score = 90 + full + p/100  
        return score

    def check_three_of_a_kind(self, hand,letters,numbers,rnum,rlet):
        cards = []
        for i in numbers:
            if numbers.count(i) == 3:
                three = i
            else: 
                cards.append(i)
        score = 45 + three + max(cards) + min(cards)/1000
        return score

    def check_two_pair(self, hand,letters,numbers,rnum,rlet):
        pairs = []
        cards = []
        for i in numbers:
            if numbers.count(i) == 2:
                pairs.append(i)
            elif numbers.count(i) == 1:
                cards.append(i)
                cards = sorted(cards,reverse=True)
        score = 30 + max(pairs) + min(pairs)/100 + cards[0]/1000
        return score

    def check_pair(self, hand,letters,numbers,rnum,rlet):    
        pair = []
        cards  = []
        for i in numbers:
            if numbers.count(i) == 2:
                pair.append(i)
            elif numbers.count(i) == 1:    
                cards.append(i)
                cards = sorted(cards,reverse=True)
        score = 15 + pair[0] + cards[0]/100 + cards[1]/1000 + cards[2]/10000
        return score

     ############# ###########################################################################################


    def score_hand(self, hand):
        """
        build card combinations in given size   
        Input : hand(of 5 cards)
        Output : score(pure score of the hand), hand_with_score(hand type such as pair vs with correspoding score in string)
        """
        letters = [hand[i][:1] for i in range(5)] # We get the suit for each card in the hand
        numbers = [int(hand[i][1:]) for i in range(5)]  # We get the number for each card in the hand
        rnum = [numbers.count(i) for i in numbers]  # We count repetitions for each number
        rlet = [letters.count(i) for i in letters]  # We count repetitions for each letter
        dif = max(numbers) - min(numbers) # The difference between the greater and smaller number in the hand
        handtype = ''
        score = 0
        hand_with_score = ''
        if 5 in rlet:
            if numbers ==[14,13,12,11,10]:
                handtype = 'royal_flush'
                score = 135
                hand_with_score = hand_with_score + handtype + ' with score : ' + str(score)

            elif dif == 4 and max(rnum) == 1:
                handtype = 'straight_flush'
                score = 120 + max(numbers)
                hand_with_score = hand_with_score + handtype + ' with score : ' + str(score)

            elif 4 in rnum:
                handtype == 'four of a kind'
                score = self.check_four_of_a_kind(hand,letters,numbers,rnum,rlet)
                hand_with_score = hand_with_score + handtype + ' with score : ' + str(score)
 
            elif sorted(rnum) == [2,2,3,3,3]:
                handtype == 'full house'
                score = self.check_full_house(hand,letters,numbers,rnum,rlet)
                hand_with_score = hand_with_score + handtype + ' with score : ' + str(score)

            elif 3 in rnum:
                handtype = 'three of a kind'
                score = self.check_three_of_a_kind(hand,letters,numbers,rnum,rlet)
                hand_with_score = hand_with_score + handtype + ' with score : ' + str(score)

            elif rnum.count(2) == 4:
                handtype = 'two pair'
                score = self.check_two_pair(hand,letters,numbers,rnum,rlet)
                hand_with_score = hand_with_score + handtype + ' with score : ' + str(score)

            elif rnum.count(2) == 2:
                handtype = 'pair'
                score = self.check_pair(hand,letters,numbers,rnum,rlet)
                hand_with_score = hand_with_score + handtype + ' with score : ' + str(score)

            else:
                handtype = 'flush'
                score = 75 + max(numbers)/100
                hand_with_score = hand_with_score + handtype + ' with score : ' + str(score)
 
        elif 4 in rnum:
            handtype = 'four of a kind'
            score = self.check_four_of_a_kind(hand,letters,numbers,rnum,rlet)
            hand_with_score = hand_with_score + handtype + ' with score : ' + str(score)

        elif sorted(rnum) == [2,2,3,3,3]:
            handtype = 'full house'
            score = self.check_full_house(hand,letters,numbers,rnum,rlet)
            hand_with_score = hand_with_score + handtype + ' with score : ' + str(score)

        elif 3 in rnum:
            handtype = 'three of a kind' 
            score = self.check_three_of_a_kind(hand,letters,numbers,rnum,rlet)
            hand_with_score = hand_with_score + handtype + ' with score : ' + str(score)

        elif rnum.count(2) == 4:
            handtype = 'two pair'
            score = self.check_two_pair(hand,letters,numbers,rnum,rlet)
            hand_with_score = hand_with_score + handtype + ' with score : ' + str(score)

        elif rnum.count(2) == 2:
            handtype = 'pair'
            score = self.check_pair(hand,letters,numbers,rnum,rlet)
            hand_with_score = hand_with_score + handtype + ' with score : ' + str(score)

        elif dif == 4:
            handtype = 'straight'
            score = 65 + max(numbers)
            hand_with_score = hand_with_score + handtype + ' with score : ' + str(score)

        else:
            handtype= 'high card'
            n = sorted(numbers,reverse=True)
            score = n[0] + n[1]/100 + n[2]/1000 + n[3]/10000 + n[4]/100000
            hand_with_score = hand_with_score + handtype + ' with score : ' + str(score)
                    
        return score, hand_with_score





