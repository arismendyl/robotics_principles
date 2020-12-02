# -*- coding: utf-8 -*-
"""
Created on Tue Dec  1 18:56:43 2020

@author: luari
"""

import numpy as np  # for array stuff and random
from PIL import Image  # for creating visual of our env
import cv2
import matplotlib.pyplot as plt  # for graphing our mean rewards over time
import pickle  # to save/load Q-Tables
from matplotlib import style  # to make pretty charts because it matters.
import time  # using this to keep track of our saved Q-Tables.
style.use("ggplot")  # setting our style!


obst_car = []
obst =[2,17,32,33,34,38,39,40,42,47,48,49,53,54,55,62,
    63,64,68,69,70,77,78,79,86,105,106,107,108,109,
    110, 112,114,115,116,117,118,119,144,151,153,156,
    159,161,163,174,181,186,189,204,206,208,219]

Player_X = 6
Player_Y = 0
Food_X = 6
Food_Y = 5
SIZE = 15
SHOW_EVERY = 100

for i in obst:
  node = i
  x = int(node % SIZE)
  y = int((node - x) / SIZE)
  obst_car.append([x,y])

class Blob:
    def __init__(self, x=None, y=None):
        if x == None or y == None:
            self.x = np.random.randint(0, SIZE)
            self.y = np.random.randint(0, SIZE)
        else:
            self.x = x
            self.y = y

    def __str__(self):
        return f"Blob ({self.x}, {self.y})"

    def __sub__(self, other):
        return (self.x-other.x, self.y-other.y)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def action(self, choice):
        '''
        Gives us 4 total movement options. (0,1,2,3)
        '''
        if choice == 0: # Up 
            self.move(x=0, y=1)
        elif choice == 1: # Down
            self.move(x=0, y=-1)
        elif choice == 2: # Left
            self.move(x=-1, y=0)
        elif choice == 3: # Right
            self.move(x=1, y=0)

    def move(self, x=False, y=False):
        bx=self.x 
        by=self.y  
        # If no value for x, move randomly
        self.x += x
    
        # If no value for y, move randomly
        self.y += y
    
    
        # If we are out of bounds, fix!
        if self.x < 0:
            self.x = 0
        elif self.x > SIZE-1:
            self.x = SIZE-1
        if self.y < 0:
            self.y = 0
        elif self.y > SIZE-1:
            self.y = SIZE-1
        
        #   # Obstacles definition
        if [self.x,self.y] in obst_car:
            self.x = bx
            self.y = by

class BlobEnv:
    SIZE = 15
    EPS_STEPS = 200
    RETURN_IMAGES = False
    MOVE_PENALTY = 10
    ENEMY_PENALTY = MOVE_PENALTY*EPS_STEPS
    FOOD_REWARD = 2_500
    OBSERVATION_SPACE_VALUES = (SIZE, SIZE, 3)  # 4
    ACTION_SPACE_SIZE = 4
    PLAYER_N = 1  # player key in dict
    FOOD_N = 2  # food key in dict
    ENEMY_N = 3  # enemy key in dict
    # the dict! (colors)
    d = {1: (255, 175, 0),
         2: (0, 255, 0),
         3: (0, 0, 255)}

    def reset(self):
        self.player = Blob(Player_X, Player_Y)
        self.food = Blob(Food_X, Food_Y)
        self.enemies = []

        for i in range(len(obst_car)):
            self.enemies.append(Blob(obst_car[i][0],obst_car[i][1]))

        self.episode_step = 0

        if self.RETURN_IMAGES:
            observation = np.array(self.get_image())
        else:
            observation = (self.player.x,self.player.y)
        return observation

    def step(self, action):
        self.episode_step += 1
        self.player.action(action)

        if self.RETURN_IMAGES:
            new_observation = np.array(self.get_image())
        else:
            new_observation = (self.player.x,self.player.y)

        obs = self.player - self.food

        if self.player == self.food:
            reward = self.FOOD_REWARD
        else:
            reward = -self.MOVE_PENALTY - 0.5*(np.abs(obs[0]) + np.abs(obs[1]))

        done = False

        if reward == self.FOOD_REWARD or self.episode_step >= self.EPS_STEPS:
            done = True
        return new_observation, reward, done

    def render(self):
        img = self.get_image()
        img = img.resize((300, 300))  # resizing so we can see our agent in all its glory.
        cv2.imshow('img',np.array(img))  # show it!
        cv2.waitKey(100)

    # FOR CNN #
    def get_image(self):
        env = np.zeros((self.SIZE, self.SIZE, 3), dtype=np.uint8)  # starts an rbg of our size
        env[self.food.x][self.food.y] = self.d[self.FOOD_N]  # sets the goal location tile to green color
        for i in self.enemies: # sets the enemies location to red
                env[i.x][i.y] = self.d[self.ENEMY_N]
        env[self.player.x][self.player.y] = self.d[self.PLAYER_N]  # sets the player tile to blue
        img = Image.fromarray(env, 'RGB')  # reading to rgb. Apparently. Even tho color definitions are bgr. ???
        return img

#%%
# Load environment, print information and render
# Actions are encoded as:  0 – Left(ish), 1 – Down(ish), 2 – Right(ish), 3 – Up(ish)
env = BlobEnv()
print("Action space: ", env.ACTION_SPACE_SIZE)
print("Observation space: ", env.OBSERVATION_SPACE_VALUES)
env.reset()
env.render()

#%%
#Q-Learning agent
Q={}
#Q-table
for i in range(0, env.OBSERVATION_SPACE_VALUES[0]):
  for ii in range(0, env.OBSERVATION_SPACE_VALUES[0]):
    Q[((i, ii))] = [np.random.uniform(0, 0) for i in range(4)]
#Q-Learning parameters

alpha = 0.8
gamma = 0.96
epsilon  = 0.5
steps_list = []
rewards_list = []
epis = 5_000
episode_rewards = []
wins = 0
FOOD_REWARD = 2_500
for i in range(epis):
    episode_reward = 0
    # Reset environment
    state = env.reset()
    step = 0
    done = False
    #Update e-greedy value
    if (i % 1000 == 999):
         epsilon*=0.9
         epsilon = max(epsilon, 0.01)
    
    #Run episode
    while not done:
        step+=1
        #Explore
        if np.random.uniform(0, 1) < epsilon:
            action = np.random.randint(3)
        else:
        #Or Exploit
            action = np.argmax(Q[state])
        #Get new state & reward from environment
        new_state, reward, done = env.step(action)
        #Update Q-Table with new knowledge
        old = Q[state][action]
        new = np.max(Q[new_state])
        Q[state][action] = (1-alpha)*old + alpha*(reward + gamma*new)
        state = new_state
        episode_reward += reward
        if i % SHOW_EVERY == 0:
            env.render()
    if i % SHOW_EVERY == 0:
        print(f"on #{i}, epsilon is {epsilon}")
        print(f"{SHOW_EVERY} ep mean: {np.mean(episode_rewards[-SHOW_EVERY:])}")
    #Update metrics
    steps_list.append(step)
    rewards_list.append(reward)
    if reward == FOOD_REWARD:
        wins += 1
    episode_rewards.append(episode_reward)

print("Done training")

#%%
moving_avg = np.convolve(episode_rewards, np.ones((SHOW_EVERY,))/SHOW_EVERY, mode='valid')

plt.plot([i for i in range(len(moving_avg))], moving_avg)
plt.ylabel(f"Reward {SHOW_EVERY}ma")
plt.xlabel("episode #")
plt.show()    
print("Final Values Q-Table")
print (Q)
print("Win percentage (total): {0}%".format(sum(rewards_list)*100/epis))
print("Win percentage (last 100 episodes): {0}%".format(sum(rewards_list[-100:])))
print("Average number of steps: {0}".format(sum(steps_list)/epis))
print("Average number of steps (last 100 episodes): {0}".format(sum(steps_list[-100:])/100))

