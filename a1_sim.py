from pybullet_envs.minitaur.envs_v2 import locomotion_gym_env
import a1_setup
import time
import rospy

a1_setup.load_sim_config(True)
env = locomotion_gym_env.LocomotionGymEnv()
env.reset()

while True:
    env.step(env.robot.set_torque[0])
    time.sleep(0.005)