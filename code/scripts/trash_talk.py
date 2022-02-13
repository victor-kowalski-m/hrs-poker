#!/usr/bin/env python

'''
This script is responsible for the trash talk part. 
It activates the speech recognition of NAO, lets him choose an appropriate reaction, propagates the action of the opponent to the poker script and allows us to let NAO speak.

Callable functions:
on_exit(): unsubscribe ALProxy memory and listener
listen(): listen to opponent, recognize his move or trash talk
talk(trigger): triggers reaction of NAO on recognized words. To not use a reaction twice, it is removed from dictionary after NAO said it. Reactions are selected randomly
speak(say_words): NAO will say 'say_words'
'''

from naoqi import ALProxy

import time
import sys
import random
import qi
import atexit


# define robot vaiables 
IP = "10.152.246.171"
PORT = 9559

memory = None
n="Listener"
asr = None
#define the ALProxy for speech recognition
asr = ALProxy("ALSpeechRecognition", IP, PORT)
asr.setLanguage("English") # set language to english

# create qi session
session = qi.Session()
try:
    session.connect("tcp://" + str(IP) + ":" + str(PORT))
except RuntimeError:
    print ("Can't connect to Naoqi at ip \n"
            "Please check your script arguments. Run with -h option for help.")
    sys.exit(1)

audio = session.service("ALAudioDevice")
audio.muteAudioOut(False)
audio.setOutputVolume(100)
audio.flushAudioOutputs()

# disable automatic movement of NAO
life_service = session.service("ALAutonomousMoves")
life_service.setExpressiveListeningEnabled(False)
asr.setAudioExpression(True)

# NAO wake up
motionProxy = ALProxy("ALMotion", IP, PORT)
motionProxy.wakeUp()


# dictionary of all of NAOs possible reactions in the game, depending on different triggers - the heart of the trash talk
trigger_dict = {
    "beginning": ["Get your popcorn ready, 'cause I'm about to put on a show.", "Look at these cards! I'll show you my hand and still win.", "I will take everything you own, down to your underwear.", "Woohoo"],
    "me all in": ["Let's see if you can handle this. I go all in!", "I go all in.", "All in baby!", "All in."],
    "won": ["Did you really think you could beat a machine?", "Trust me: The pain will go away one day", "Do you need a tissue?", "Maybe you will win in love instead."],
    "lost": ["Oh man, I thought we were playing blackjack.", "If there weren't luck involved, I would win every time.", "Enjoy the company of my chips as long as you can.", "Damn, you play better than you look."],
    "reaction mean": ["Bring it, don't sing it.", "I've beaten bigger guys on my way to a fight."],
    "reaction sad": ["Get better, not bitter.", "Don't worry. It'll all be over soon."],
    "begin second game + won last": ["Do me a favour this time and TRY to lose with some dignity.", "Okay, but once you start crying, we'll stop.", "You seem to really want to get your ass kicked again."],
    "begin second game + lost last": ["I was giving you a head start. Now I'm starting to play for real.", "Last time was only luck. This time will be different.", "You had beginners luck. This won't work twice."],
    "random": ["You can't beat me.", "Your mom called. She says you left your game at home.", "You should show the other player some love, because I'm about to destroy him.", "Can't read my, can't read my No, he can't read my poker face Po-po-po-poker face, po-po-poker face."]
}


def on_exit():
    '''
    unsubscribe speech recognition and memory on exit
    Otherwise, NAO gets stuck, memory overflows and a restart is necessary 
    '''

    try:
        asr.unsubscribe(n)
    except:
        pass
    try:
        memory.unsubscribeToEvent("WordRecognized", n)
    except:
        pass

atexit.register(on_exit)

def listen():
    '''
    responsible for understanding predifined words in 'vocabulary'
    NAO listends to opponent and is supposed to understand his moves, which are then propagated to the poker script.
    NAO is also supposed to react to certain sentences of opponent, which can be either mean or sad.
    Input: -
    Output: list of strings - Recognized word(s)
    '''

    global asr, trigger_dict, memory
    spoken_words = [] #initialize list for recognized words
    
    try:
        asr.unsubscribe(n)
    except:
        pass
    asr.pause(True)


    # define words that NAO should be able to understand 
    numbers = ["Five", "Ten", "Fifteen", "Twenty"] # money input
    action_one = ["Check", "Fold", "All in"] # opponent actions
    opp_reaction_bad = ["You will lose", "You suck"] # NAOs reactions
    opp_reaction_sad = ["I am so bad", "I am losing"] # NAOs reactions
    vocabulary = numbers + opp_reaction_bad + opp_reaction_sad + action_one + ["Raise"] #add all together to create one big vocabulary
    asr.setVocabulary(vocabulary, False)


    try:
        memory = ALProxy("ALMemory", IP, PORT)
        memory.subscribeToEvent("WordRecognized", n, "onWordRecognized")
        memory.declareEvent(n)
    
    except Exception:
        print("error")
        sys.exit(1)


    # start word recognition:
    print("speech recognition started...")

    ts = time.time()
    
    # listen for 2 seconds, until right words are recognized or 20s are over
    while len(spoken_words)<=1 and (time.time()-ts)<=20:
        asr.subscribe(n)
        time.sleep(3)
        words = memory.getData("WordRecognized")[0] #get recognized words 
        confidence = memory.getData("WordRecognized")[1] #get confidence of recognized words
        if confidence>=0:
            print("Machine's confidence: ", confidence)
        
        if confidence <= 0.4:
            # if confidence of an recognized word is below 0.4, do not use it
            spoken_words = []

        else:
            # if confidence is above, continue
            print("you said: ", words)
            '''
            here we distinguish between three cases:
            1) opponent said something mean - NAO reacts. Keep spoken_words empty -> after NAOs reaction, continue listening
            2) Opponent said something sad - NAO reacts. Keep spoken_words empty -> after NAOs reaction, continue listening
            3) Opponent said his action or a number he would like to raise - add to spoken_words 
            '''
            if words in opp_reaction_bad:
                talk("reaction mean")
                words = []

            elif words in opp_reaction_sad:
                talk("reaction sad")
                words = []

            elif (words in action_one) or (words in numbers):
                # only for 'raise' and amount of chips, two words should be recognized. If only a number is recognized, we assume he had problems with 'raise' and continue anyways as if he recognized 'raise' before
                spoken_words = words
                break 

            if len(words)>=1:
                spoken_words.append(words) 


    print("all words: ", spoken_words)
    asr.unsubscribe(n)
    memory.unsubscribeToEvent("WordRecognized", n)

    return spoken_words

def talk(trigger):
    '''
    react to trigger and remove reaction from dictionary
    Input: str - key of dictionary: trigger_dict
    Output: -
    '''
    global trigger_dict
    tts = ALProxy("ALTextToSpeech", IP, PORT)
    tts.setLanguage("English") # set language to english
    tts.setParameter("speed", 10) # manage speed  


    # randomly choose entry of list, that correspponds to key, except list is already empty
    if len(trigger_dict[trigger]) >= 1:
        i = random.randint(0,len(trigger_dict[trigger])-1) 
        answer = trigger_dict[trigger][i]
        print("Machine says: ", answer)

    # if list at key is not empty, NAO speaks
    if trigger_dict[trigger] != []:
        tts.say(answer) # NAO talks
        trigger_dict[trigger].pop(i) # remove reaction from dictionary

def say(say_words):
    '''
    say() functions only as a talking command for NAO. 
    Input: str -  a sentence NAO is supposed say
    Output: -
    '''

    tts = ALProxy("ALTextToSpeech", IP, PORT)
    tts.setLanguage("English") # set language to english
    tts.setParameter("speed", 10) # manage speed
    tts.say(say_words)

