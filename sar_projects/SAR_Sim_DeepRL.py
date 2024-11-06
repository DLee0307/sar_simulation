import gymnasium as gym
from gymnasium import spaces

import numpy as np
from threading import Thread,Event

import rclpy
import csv
import random
import time
import math

import sys
import os


#SET PYTHONPATH 
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append('/home/dlee/ros2_ws/src/sar_simulation')

from sar_env import SAR_Sim_Interface

EPS = 1e-6 # Epsilon (Prevent division by zero)
YELLOW = '\033[93m'
RED = '\033[91m'
GREEN = '\033[92m'
BLUE = '\033[34m'  
RESET = '\033[0m'  # Reset to default color

class SAR_Sim_DeepRL(SAR_Sim_Interface,gym.Env):

    def __init__(self,Ang_Acc_range=[-90,0],V_mag_range=[1.5,3.5],V_angle_range=[5,175],Plane_Angle_range=[0,0],Render=True,Fine_Tune=True,GZ_Timeout=False):
        SAR_Sim_Interface.__init__(self, GZ_Timeout=GZ_Timeout)
        gym.Env.__init__(self)

        if self.Policy_Type != "DEEP_RL_SB3":
            str_input = self.userInput(YELLOW + "Incorrect Policy Activated. Continue? (y/n): " + RESET,str)
            if str_input.lower() == 'n':
                raise Exception('[ERROR] Incorrect Policy Type Activated')
            else:
                pass

        ######################
        #    GENERAL CONFIGS
        ######################
        
        ## ENV CONFIG SETTINGS
        self.Env_Name = "SAR_Sim_DeepRL_Env"

        ## TESTING CONDITIONS     
        self.V_mag_range = V_mag_range  
        self.V_angle_range = V_angle_range
        self.Plane_Angle_range = Plane_Angle_range
        self.setAngAcc_range(Ang_Acc_range)

        self.Fine_Tuning_Flag = Fine_Tune
        self.TestCondition_idx = 0
        self.TestCondition_Wrap = False
        self.TestingConditions = []

        if self.Fine_Tuning_Flag:
            
            ## LOAD TESTING CONDITIONS
            self.BASE_PATH = "/home/dlee/ros2_ws/src/sar_simulation"
            csv_file_path = f"{self.BASE_PATH}/sar_projects/DeepRL/Training_Conditions/TrainingConditions_{int(self.Plane_Angle_range[1])}deg.csv"

            with open(csv_file_path, mode='r') as csv_file:
                csv_reader = csv.DictReader(csv_file)
                for row in csv_reader:
                    self.TestingConditions.append((float(row['Plane_Angle']),float(row['V_angle']),float(row['V_mag']),))

            ## SHUFFLE TESTING CONDITIONS
            random.shuffle(self.TestingConditions)

        ## TIME CONSTRAINTS
        self.t_rot_max = np.sqrt(np.radians(360)/np.max(np.abs(self.Ang_Acc_range))) # Allow enough time for a full rotation [s]
        self.t_impact_max = 1.5     # [s]
        self.t_ep_max = 5.0         # [s]
        self.t_real_max = 5*60      # [s]

        ## INITIAL LEARNING/REWARD CONFIGS
        self.Initial_Step = False
        self.K_ep = 0
        self.Pol_Trg_Threshold = 0.5
        self.Done = False
        self.reward = 0
        self.reward_vals = np.array([0,0,0,0,0,0,0])
        self.reward_weights = {
            "W_Dist":0.4,
            "W_tau_cr":0.1,
            "W_tx":1.0,
            "W_LT":1.0,
            "W_GM":1.0,
            "W_Phi_rel":2.0,
            "W_Legs":2.0
        }
        self.W_max = sum(self.reward_weights.values())

        self.D_perp_CR_min = np.inf
        self.D_perp_pad_min = np.inf
        self.Tau_CR_trg = np.inf
        self.Tau_trg = np.inf

        ## DOMAIN RANDOMIZATION
        self.Mass_std = 0.00*self.Ref_Mass
        self.Iyy_std = 0.00*self.Ref_Iyy

        ## DEFINE OBSERVATION SPACE
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(4,), dtype=np.float32)
        self.obs_trg = np.zeros(self.observation_space.shape,dtype=np.float32) # Obs values at triggering

        ## DEFINE ACTION SPACE
        self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
        self.action_trg = np.zeros(self.action_space.shape,dtype=np.float32) # Action values at triggering


if __name__ == "__main__":

    rclpy.init()

    env = SAR_Sim_DeepRL(Ang_Acc_range=[-90,0],V_mag_range=[1.5,3.5],V_angle_range=[5,175],Plane_Angle_range=[0,0],Render=True,Fine_Tune=False)

    rclpy.shutdown()