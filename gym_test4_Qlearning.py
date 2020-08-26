

import gym
import gym_jaco
import numpy as np

env = gym.make('jaco-v0')

#print(env.observation_space.high)
#print(env.observation_space.low)

#print(env.action_space.high)
#print(env.action_space.low)

LEARNING_RATE = 0.1
DISCOUNT = 0.95
EPISODES = 50
Samples = 500
SHOW_EVERY=10

DiscreteNum_SizeQ = 10

DISCRETE_OS_SIZE = [DiscreteNum_SizeQ]*len(env.observation_space.high)
DISCRETE_AC_SIZE = [DiscreteNum_SizeQ]*len(env.action_space.high)


#print(DISCRETE_OS_SIZE)
#print(DISCRETE_AC_SIZE)

discrete_os_win_size = (env.observation_space.high-env.observation_space.low)/DISCRETE_OS_SIZE
discrete_ac_win_size = (env.action_space.high-env.action_space.low)/DISCRETE_AC_SIZE


#print(discrete_os_win_size)

q_table = np.random.uniform(low=-1	, high=1, size=(DISCRETE_OS_SIZE + DISCRETE_AC_SIZE))

#print(len(q_table))
#print(len(q_table[0]))
#print(q_table.shape)

def get_discrete_state(state):
	discrete_state = (state - env.observation_space.low) / discrete_os_win_size
	return tuple(discrete_state.astype(np.int))

def get_continous_action(index):
	continous_action = index*discrete_ac_win_size+env.action_space.low
	return continous_action


for episode in range(EPISODES):
	print("this is episode #:")
	print(episode)
     
	if episode % SHOW_EVERY == 0:
		#print(episode)
		render = True
	else: 
		render = True
		
	discrete_state = get_discrete_state(env.reset())

	sample = 0
	while sample < Samples:
		sample = sample+1
		index = np.unravel_index(np.argmax(q_table[discrete_state], axis=None), q_table[discrete_state].shape)
		
		if index[0] == 10:
			index[0] == 9
		if index[1] == 10:
			index[1] == 9
		if index[2] == 10:
			index[2] == 9
		
		continous_action = get_continous_action(index)
		#print(continous_action)
		new_state, reward, done, info = env.step(continous_action)
		new_discrete_state = get_discrete_state(new_state)

		if render:
			env.render()

		max_future_q = np.max(q_table[new_discrete_state])
		current_q = q_table[discrete_state + index]
		new_q = (1-LEARNING_RATE)*current_q + LEARNING_RATE * (reward + DISCOUNT * max_future_q)
		q_table[discrete_state + index] = new_q
		
		if done:
			print("Episode finished after {} timesteps".format(sample+1))
			q_table[discrete_state + index] = 0
			break
		
		discrete_state = new_discrete_state

env.close()

