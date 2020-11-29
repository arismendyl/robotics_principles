import numpy as np  # for array stuff and random
from PIL import Image  # for creating visual of our env
import cv2  # for showing our visual live
import matplotlib.pyplot as plt  # for graphing our mean rewards over time
import pickle  # to save/load Q-Tables
from matplotlib import style  # to make pretty charts because it matters.
import time  # using this to keep track of our saved Q-Tables.

style.use("ggplot")  # setting our style!


SIZE = 15
HM_EPISODES = 1000
MOVE_PENALTY = 10  # feel free to tinker with these!
ENEMY_PENALTY = 300  # feel free to tinker with these!
FOOD_REWARD = 2500  # feel free to tinker with these!
MAX_STEPS = 2000
epsilon = 0.2  # randomness
EPS_DECAY = 0.9999  # Every episode will be epsilon*EPS_DECAY
SHOW_EVERY = 10 # how often to play through env visually.

start_q_table = None  # if we have a pickled Q table, we'll put the filename of it here.

LEARNING_RATE = 0.1
DISCOUNT = 0.95

PLAYER_N = 1  # player key in dict
FOOD_N = 2  # food key in dict
ENEMY_N = 3  # enemy key in dict

# the dict! Using just for colors
d = {1: (255, 175, 0),  # blueish color
     2: (0, 255, 0),  # green
     3: (0, 0, 255)}  # red

obst_car = []
obst =[2,17,32,33,34,38,39,40,42,47,48,49,53,54,55,62,
    63,64,68,69,70,77,78,79,86,105,106,107,108,109,
    110, 112,114,115,116,117,118,119,144,151,153,156,
    159,161,163,174,181,186,189,204,206,208,219]

for i in obst:
  node = i
  x = int(node % SIZE)
  y = int((node - x) / SIZE)
  obst_car.append([x,y])

class Blob():
    def __init__(self, x=None, y=None):
        if x == None or y == None:
            self.x = np.random.randint(0, SIZE)
            self.y = np.random.randint(0, SIZE)
        else:
            self.x = x
            self.y = y
        
    def __str__(self):
        return f"{self.x}, {self.y}"
    def __sub__(self, other):
        return (self.x-other.x, self.y-other.y)
    def action(self, choice):
        '''
        Gives us 4 total movement options. (0,1,2,3)
        '''
        if choice == 0:
            self.move(x=0, y=1)
        elif choice == 1:
            self.move(x=0, y=-1)
        elif choice == 2:
            self.move(x=-1, y=0)
        elif choice == 3:
            self.move(x=1, y=0)
    def move(self, x=False, y=False):
      bx=self.x 
      by=self.y  
      # If no value for x, move randomly
      if not x:
          self.x += np.random.randint(-1, 2)
      else:
          self.x += x

      # If no value for y, move randomly
      if not y:
          self.y += np.random.randint(-1, 2)
      else:
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

player = Blob(0,0)
food = Blob(7,4)

enemies = []

for i in range(len(obst_car)):
    enemies.append(Blob(obst_car[i][0],obst_car[i][1]))


print(player)
print(food)
print(player-food)
player.move()
print(player-food)
player.action(2)
print(player-food)

if start_q_table is None:
    # initialize the q-table#
    q_table = {}
    for i in range(-SIZE+1, SIZE):
        for ii in range(-SIZE+1, SIZE):
                        q_table[((i, ii))] = [np.random.uniform(-5, 0) for i in range(4)]
else:
    with open(start_q_table, "rb") as f:
        q_table = pickle.load(f)

episode_rewards = []
wins = 0

for episode in range(HM_EPISODES):
    player = Blob(0,0)
    food = Blob(7,4)
    if episode % SHOW_EVERY == 0:
        print(f"on #{episode}, epsilon is {epsilon}")
        print(f"{SHOW_EVERY} ep mean: {np.mean(episode_rewards[-SHOW_EVERY:])}")
        show = True
    else:
        show = False
    episode_reward = 0
    for i in range(MAX_STEPS):
        obs = (player-food)
        #print(obs)
        if np.random.random() > epsilon:
            # GET THE ACTION
            action = np.argmax(q_table[obs])
        else:
            action = np.random.randint(0, 4)
        # Take the action!
        player.action(action)

        if player.x == food.x and player.y == food.y:
            reward = FOOD_REWARD
        else:
            reward = -MOVE_PENALTY - np.sqrt(obs[0]**2+obs[1]**2)**2
        new_obs = (player-food)  # new observation
        max_future_q = np.max(q_table[new_obs])  # max Q value for this new obs
        current_q = q_table[obs][action]  # current Q for our chosen action
        if reward == FOOD_REWARD:
            new_q = FOOD_REWARD
        else:
            new_q = (1 - LEARNING_RATE) * current_q + LEARNING_RATE * (reward + DISCOUNT * max_future_q)
        if show:
            env = np.zeros((SIZE, SIZE, 3), dtype=np.uint8)  # starts an rbg of our size
            env[food.x][food.y] = d[FOOD_N]  # sets the food location tile to green color
            env[player.x][player.y] = d[PLAYER_N]  # sets the player tile to blue
            for i in enemies:
                env[i.x][i.y] = d[ENEMY_N]  # sets the enemy location to red
            img = Image.fromarray(env, 'RGB')  # reading to rgb. Apparently. Even tho color definitions are bgr. ???
            img = img.resize((300, 300))  # resizing so we can see our agent in all its glory.
            cv2.imshow('image', np.array(img))  # show it!
            if reward == FOOD_REWARD:  # crummy code to hang at the end if we reach abrupt end for good reasons or not.
                if cv2.waitKey(500) & 0xFF == ord('q'):
                    break
            else:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        episode_reward += reward
        if reward == FOOD_REWARD:
            break
    if reward == FOOD_REWARD:
        wins = wins + 1
    episode_rewards.append(episode_reward)
    epsilon *= EPS_DECAY  

moving_avg = np.convolve(episode_rewards, np.ones((SHOW_EVERY,))/SHOW_EVERY, mode='valid')

plt.plot([i for i in range(len(moving_avg))], moving_avg)
plt.ylabel(f"Reward {SHOW_EVERY}ma")
plt.xlabel("episode #")
plt.show()

with open(f"qtable-{int(time.time())}.pickle", "wb") as f:
    pickle.dump(q_table, f)    