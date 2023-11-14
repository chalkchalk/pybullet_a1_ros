import pybullet as p
import time
import pybullet_data as pd
import numpy as np

dt = 1 / 500
real_time_factor = 0.5

class A1():
       
    
    def __init__(
        self, urdf, dt, real_time_factor
    ):
        self.A1_DEFAULT_ABDUCTION_ANGLE = 0
        self.A1_DEFAULT_HIP_ANGLE = 0.9
        self.A1_DEFAULT_KNEE_ANGLE = -1.8
        self.NUM_LEGS = 4
        self.INIT_MOTOR_ANGLES = np.array([
                self.A1_DEFAULT_ABDUCTION_ANGLE,
                self.A1_DEFAULT_HIP_ANGLE,
                self.A1_DEFAULT_KNEE_ANGLE
        ] * self.NUM_LEGS)
        self.MOTOR_NAMES = [
            "FR_hip_joint",
            "FR_upper_joint",
            "FR_lower_joint",
            "FL_hip_joint",
            "FL_upper_joint",
            "FL_lower_joint",
            "RR_hip_joint",
            "RR_upper_joint",
            "RR_lower_joint",
            "RL_hip_joint",
            "RL_upper_joint",
            "RL_lower_joint",
        ]
        
        self.TOE_NAMES = [
            "FR_toe_fixed",
            "FL_toe_fixed",
            "RR_toe_fixed",
            "RL_toe_fixed"
        ]
        self.motor_ids = []
        self.toe_ids = []
        self.motor_num = 12
        
        p.connect(p.GUI)
        self.dt = dt
        self.real_time_factor = real_time_factor
        p.setAdditionalSearchPath(pd.getDataPath())
        p.loadURDF("plane.urdf")
        p.setTimeStep(self.dt)
        self.robot = p.loadURDF(urdf,[0,0,0.5])
        p.setGravity(0,0,-9.8)
        
        self.motor_dir = [-1, 1, 1, 1, 1, 1, -1, 1, 1, 1, 1, 1]
        
        for j in range(p.getNumJoints(self.robot)):
            joint_info = p.getJointInfo(self.robot,j)
            name = joint_info[1].decode('utf-8')
            print("joint_info[1]=",name)
            if name in self.MOTOR_NAMES:
                self.motor_ids.append(j)
            if name in self.TOE_NAMES:
                self.toe_ids.append(j)
        
        
        
        for index in range(len(self.motor_ids)):
            p.resetJointState(self.robot, self.motor_ids[index], self.INIT_MOTOR_ANGLES[index])
            p.setJointMotorControl2(self.robot, self.motor_ids[index], p.POSITION_CONTROL, force=0)
    
    def getBasePosition(self):
        position, orientation = p.getBasePositionAndOrientation(self.robot)
        return position

    def get_base_orientation(self):
        position, orientation = p.getBasePositionAndOrientation(self.robot)
        return orientation
    
    def get_motor_angles(self):
        motor_angles = []
        for i in range(self.motor_num):
            joint_states = p.getJointState(self.robot, self.motor_ids[i])
            motor_angles.append(joint_states[0])
        motor_angles = np.multiply(motor_angles, self.motor_dir)
        return motor_angles
    
    def getMotorVelocities(self):
        motor_velocities = []
        for i in range(self.motor_num):
            joint_states = p.getJointState(self.robot, self.motor_ids[i])
            motor_velocities.append(joint_states[1])
        motor_velocities = np.multiply(motor_velocities, self.motor_dir)
        return motor_velocities

    def getMotorTorques(self):
        motor_torques = []
        for i in range(self.motor_num):
            joint_states = p.getJointState(self.robot, self.motor_ids[i])
            motor_torques.append(joint_states[3])
        motor_torques = np.multiply(motor_torques, self.motor_dir)
        return motor_torques

    def get_foot_contact_forces(self):
        all_contacts = p.getContactPoints(bodyA=self.robot)
        contact_forces = [0, 0, 0, 0]
        for contact in all_contacts:
            link_a_name = contact[3]
            if link_a_name in self.toe_ids:
                normal_force = contact[9]
                contact_forces[self.toe_ids.index(link_a_name)] = normal_force
        #TODO add friction force
        return contact_forces


    def simulation_step(self):
        p.stepSimulation()
        time.sleep(self.dt / self.real_time_factor)

if __name__ == "__main__" :
    a1 = A1("a1/a1.urdf", dt, real_time_factor)
    while True:
        a1.simulation_step()
    