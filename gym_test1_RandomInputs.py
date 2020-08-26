

import gym
import gym_jaco
env = gym.make('jaco-v0')
env.reset()
for _ in range(1000):
    env.render()
    env.step(env.action_space.sample()) # take a random action
    #observation, reward, done, info = env.step([1, 1, 1]) # take a random action
    
env.close()