

from gym.envs.registration import register

register(
    id='jaco-v0',
    entry_point='gym_jaco.envs:JacoEnv',
)