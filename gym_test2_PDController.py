

import gym
import gym_jaco


env = gym.make('jaco-v0')


# gain for controller
K_p = 10
k_d = 1
# sample freq
dt = 1./240.
# counting sample points
sample = 0
# sample upper bound
Samples = 100
# counting episode
episode = 0
#  episode upper bound
Episodes = 10

# target info
target = [0.65, 0, 0.5]

for episode in range(Episodes):
    # p.startStateLogging(STATE_LOGGING_VIDEO_MP4)
    # read the end-effector position xyz
    observation = env.reset()

    for sample in range (Samples):
        env.render()
        delta_x = target[0]-observation[0]
        delta_y = target[1]-observation[1]
        delta_z = target[2]-observation[2]

        Gain_pdx = K_p*delta_x + k_d*delta_x/dt
        Gain_pdy = K_p*delta_y + k_d*delta_y/dt
        Gain_pdz = K_p*delta_z + k_d*delta_z/dt 
        
        action = [Gain_pdx, Gain_pdy, Gain_pdz]

        observation, reward, done, info = env.step(action)

        if done:
            print("Episode finished after {} timesteps".format(sample+1))
            break
env.close()



