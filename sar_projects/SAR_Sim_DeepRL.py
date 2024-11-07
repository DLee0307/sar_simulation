import gymnasium as gym
from gymnasium import spaces
# Qt Issue : pippip uninstall opencv-python pip install opencv-python-headless
# https://github.com/NVlabs/instant-ngp/discussions/300

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

    def __init__(self,Ang_Acc_range=[-90,0],V_mag_range=[1.5,3.5],V_angle_range=[5,175],Plane_Angle_range=[0,180],Render=True,Fine_Tune=True,GZ_Timeout=False):
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

        self._start_RealTimeTimer()

    def _start_RealTimeTimer(self):
        
        self.start_time_real = time.time()
        RealTimeTimer_thread = Thread(target=self._RealTimeTimer)
        RealTimeTimer_thread.daemon = True
        RealTimeTimer_thread.start()

    def _RealTimeTimer(self):

        ## IF REAL TIME EXCEEDS TIMEOUT VALUE THEN RESTART SIM
        while True:
            if (time.time() - self.start_time_real > self.t_real_max) and not self.Done:
                self.Done = True
                print(YELLOW + f"Real Time Exceeded: {time.time() - self.start_time_real:.3f} s" + RESET)
                print(YELLOW + f"Sim Likely Frozen: Restarting Simulation Env" + RESET)

                self._kill_Sim()
                
            time.sleep(1)

    #!!! Need to change
    def reset(self,seed=None,options=None):

        self._wait_for_sim_running()

        self.start_time_real = time.time()
        self.resetPose()
        
        print("!!!!!!!!!!!!!!")

    def _resetParams(self):

        ######################
        #    GENERAL CONFIGS
        ######################

        ## RESET LEARNING/REWARD CONDITIONS
        self.K_ep += 1
        self.Done = False
        self.reward = 0
        self.reward_vals = np.array([0,0,0,0,0,0,0])

        self.D_perp_CR_min = np.inf
        self.D_perp_pad_min = np.inf
        self.Tau_CR_trg = np.inf
        self.Tau_trg = np.inf

        self.obs_trg = np.full(self.obs_trg.shape[0],np.nan)
        self.action_trg = np.full(self.action_trg.shape[0],np.nan)

        self.a_Trg_trg = np.nan
        self.a_Rot_trg = np.nan

    def _setTestingConditions(self):

        if self.Fine_Tuning_Flag:
            
            ## SET TESTING CONDITIONS
            Plane_Angle = self.TestingConditions[self.TestCondition_idx][0]
            V_angle_B_O = self.TestingConditions[self.TestCondition_idx][1]
            V_mag_B_O = self.TestingConditions[self.TestCondition_idx][2]

            # Add code from DH
            if np.isnan(getattr(self, 'Plane_Angle_deg', np.nan)):
                self.Plane_Angle_deg = 0

            #!!! Need to change
            ## CONVERT GLOBAL ANGLE TO RELATIVE ANGLE
            ## self._setPlanePose(self.r_P_O,Plane_Angle)
            self.V_angle = self.Plane_Angle_deg + V_angle_B_O
            self.V_mag = V_mag_B_O

            ## UPDATE TESTING CONDITION INDEX
            self.TestCondition_idx += 1
            if self.TestCondition_idx >= len(self.TestingConditions):

                if self.TestCondition_Wrap == True:
                    self.Fine_Tuning_Flag = False
                    self.TestCondition_idx = np.nan

                self.TestCondition_idx = 0
                self.TestCondition_Wrap = True
            
        else:

            ## SAMPLE SET PLANE POSE
            Plane_Angle_Low = self.Plane_Angle_range[0]
            Plane_Angle_High = self.Plane_Angle_range[1]
            Plane_Angle = np.random.uniform(Plane_Angle_Low,Plane_Angle_High)
            #self._setPlanePose(self.r_P_O,Plane_Angle)

            ## SAMPLE VELOCITY AND FLIGHT ANGLE
            #print("Sampled V_mag:", self.Plane_Angle_deg)
            V_mag,V_angle = self._sampleFlightConditions(self.V_mag_range,self.V_angle_range)
            self.V_mag = V_mag
            self.V_angle = V_angle
            
    def _initialStep(self):

        ## DOMAIN RANDOMIZATION (UPDATE INERTIAL VALUES)
        Iyy_DR = self.Ref_Iyy + np.random.normal(0,self.Iyy_std)
        Mass_DR = self.Ref_Mass + np.random.normal(0,self.Mass_std)
        #self._setModelInertia(Mass_DR,[self.Ref_Ixx,Iyy_DR,self.Ref_Izz])

        # Add code from DH
        if np.isnan(getattr(self, 'Plane_Angle_rad', np.nan)):
            self.Plane_Angle_rad = 0
        # Add code from DH
        if np.isnan(getattr(self, 'r_P_O', [np.nan])[0]):
            self.r_P_O = [3.0, 0.0, 2.5]

        ## CALC STARTING VELOCITY IN GLOBAL COORDS
        V_tx = self.V_mag*np.cos(np.deg2rad(self.V_angle))
        V_perp = self.V_mag*np.sin(np.deg2rad(self.V_angle))
        V_B_P = np.array([V_tx,0,V_perp])               # {t_x,n_p}
        V_B_O = self.R_PW(V_B_P,self.Plane_Angle_rad)   # {X_W,Z_W}

        ## CALCULATE STARTING TAU VALUE
        # self.Tau_CR_start = self.t_rot_max*np.random.uniform(0.9,1.1) # Add noise to starting condition
        self.Tau_CR_start = 0.5 + np.random.uniform(-0.05,0.05)
        try:
            self.Tau_Body_start = (self.Tau_CR_start + self.Collision_Radius/(V_perp+EPS)) # Tau read by body
        except:
            print("Exception")
        self.Tau_Accel_start = 1.0 # Acceleration time to desired velocity conditions [s]

        ## CALC STARTING POSITION IN GLOBAL COORDS
        # (Derivation: Research_Notes_Book_3.pdf (9/17/23))

        r_P_O = np.array(self.r_P_O)                                        # Plane Position wrt to Origin - {X_W,Z_W}
        r_P_B = np.array([(self.Tau_CR_start + self.Tau_Accel_start)*V_tx,
                          0,
                          (self.Tau_Body_start + self.Tau_Accel_start)*V_perp])  # Body Position wrt to Plane - {t_x,n_p}
        r_B_O = r_P_O - self.R_PW(r_P_B,self.Plane_Angle_rad)    

        ## LAUNCH QUAD W/ DESIRED VELOCITY
        self.initial_state = (r_B_O,V_B_O)
        self.Sim_VelTraj(pos=r_B_O,vel=V_B_O)
        self._iterStep(n_steps=1000)

        #self._getTick()
        self._getObs()






if __name__ == "__main__":

    rclpy.init()

    env = SAR_Sim_DeepRL(Ang_Acc_range=[-90,0],V_mag_range=[1.5,3.5],V_angle_range=[5,175],Plane_Angle_range=[0,180],Render=True,Fine_Tune=False)

    env._setTestingConditions()
    env._initialStep()    

    rclpy.spin(env.node)

    #rclpy.shutdown()