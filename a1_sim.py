from pybullet_envs.minitaur.envs_v2 import locomotion_gym_env
import a1_setup
import time
import pybullet as p

a1_setup.load_sim_config(True)
env = locomotion_gym_env.LocomotionGymEnv()
env.reset()
joint_infos = []
for j in range(p.getNumJoints(1)):
        joint_info = p.getJointInfo(1,j)
        print(joint_info)
        joint_infos.append(joint_info)
#12 13, 2  3, 7 8, 17 18
# FR_hip_fixed FR_upper_joint

while True:
    hybrid_action = [0.1 for i in range(12)]
    env.step(hybrid_action)
    time.sleep(0.005)