from pybullet_envs.minitaur.envs_v2 import locomotion_gym_env
import a1_setup
import time
import pybullet as p

a1_setup.load_sim_config(True)
env = locomotion_gym_env.LocomotionGymEnv()
env.reset()

while True:
    hybrid_action = [0 for i in range(12)]
    env.step(hybrid_action)
    
    time.sleep(0.005)