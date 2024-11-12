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
        #print("r_P_B : ", r_P_B)

        ## DESIRED ACCELERATION VALUES
        Acc = self.TrajAcc_Max
    
        a_x = Acc[0]
        a_z = Acc[2]

        ## CALC OFFSET POSITIONS
        t_x = V_B_O[0]/a_x    # Time required to reach Vx
        t_z = V_B_O[2]/a_z    # Time required to reach Vz

        x_0 = r_B_O[0] - V_B_O[0]**2/(2*a_x) - V_B_O[0]*t_z     # X-position Vel reached
        y_0 = r_B_O[1]                                          # Y-position Vel reached
        z_0 = r_B_O[2] - V_B_O[2]**2/(2*a_z)                    # Z-position Vel reached  

        r_B_O = np.array([x_0,y_0,z_0])

        ## LAUNCH QUAD W/ DESIRED VELOCITY
        self.initial_state = (r_B_O,V_B_O)
        self.Sim_VelTraj(pos=r_B_O,vel=V_B_O)
        self._iterStep(n_steps=1000)
        
        # Add code from DH : Beacause lock step?
        time.sleep(5)

        ## ROUND OUT STEPS TO BE IN SYNC WITH CONTROLLER
        if self._getTick()%10 != 0:
            n_steps = 10 - (self._getTick()%10)
            self._iterStep(n_steps=n_steps)

        ## RESET/UPDATE TIME CONDITIONS
        self.start_time_ep = self._getTime()
        print("_getTime:", self.start_time_ep)
        self.start_time_trg = np.nan
        self.start_time_impact = np.nan
        self.t_flight_max = self.Tau_Body_start*2.0   # [s]
        self.t_trg_max = self.Tau_Body_start*2.5 # [s]

        # print("self.SAR_Type", self.SAR_Type)
        # print("self.SAR_Config", self.SAR_Config)
        # print("self.Policy_Type", self.Policy_Type)

    def step(self, action):

        # 1. TAKE ACTION
        # 2. UPDATE STATE
        # 3. CALC REWARD
        # 4. CHECK TERMINATION
        # 5. RETURN VALUES

        self._wait_for_sim_running()

        ## ROUND OUT STEPS TO BE IN SYNC WITH CONTROLLER
        if self._getTick()%10 != 0:
            n_steps = 10 - (self._getTick()%10)
            self._iterStep(n_steps=n_steps)

        a_Trg = action[0]
        a_Rot = self.scaleValue(action[1],original_range=[-1,1], target_range=self.Ang_Acc_range)

        if self.Policy_Type != "DEEP_RL_SB3":
            a_Trg = 1.0

        ########## POLICY PRE-TRIGGER ##########
        if a_Trg <= self.Pol_Trg_Threshold:

            ## 2) UPDATE STATE
            self._iterStep(n_steps=10)
            t_now = self._getTime()

            # UPDATE RENDER
            self.render()

            # GRAB NEXT OBS
            next_obs = self._getObs()

            # 3) CALCULATE REWARD
            reward = 0.0

            # 4) CHECK TERMINATION/TRUNCATED

            # ============================
            ##    Termination Criteria 
            # ============================

            if self.Done == True:
                self.error_str = "Episode Completed: Done [Terminated]"
                terminated = True
                truncated = False
                # print(YELLOW,self.error_str,RESET)
            
            ## IMPACT TERMINATION
            elif self.Impact_Flag_Ext == True:
                self.error_str = "Episode Completed: Impact [Terminated]"
                terminated = True
                truncated = False
                # print(YELLOW,self.error_str,RESET)

            ## EPISODE TIMEOUT
            elif (t_now - self.start_time_ep) > self.t_flight_max:
                self.error_str = "Episode Completed: Time Exceeded [Truncated]"
                terminated = False
                truncated = True
                # print(YELLOW,self.error_str,RESET)

            ## REAL-TIME TIMEOUT
            elif (time.time() - self.start_time_real) > self.t_real_max:
                self.error_str = "Episode Completed: Episode Time Exceeded [Truncated] "
                terminated = False
                truncated = True
                # print(YELLOW,self.error_str,f"{(time.time() - self.start_time_real):.3f} s",RESET)

            else:
                terminated = False
                truncated = False

            info_dict = {
                "reward_vals": self.reward_vals,
                "reward": reward,
                "a_Rot": a_Rot,
                "Trg_Flag": self.Trg_Flag,
                "Impact_Flag_Ext": self.Impact_Flag_Ext,
                "D_perp_pad_min": self.D_perp_pad_min,
                "Tau_CR_trg": self.Tau_CR_trg,
                "Plane_Angle": self.Plane_Angle_deg,
                "V_mag": self.V_mag,
                "V_angle": self.V_angle,
                "TestCondition_idx": self.TestCondition_idx,
            }
            
            # 5) RETURN VALUES
            return(
                next_obs,
                reward,
                terminated,
                truncated,
                info_dict,
            )

        ########## POLICY POST-TRIGGER ##########
        elif a_Trg >= self.Pol_Trg_Threshold:
            print("**************************************")

            # GRAB TERMINAL OBS/ACTION
            self.obs_trg = self._getObs()
            self.action_trg = action

            # 2) FINISH EPISODE
            self.start_time_trg = self._getTime()
            terminated,truncated = self._finishSim(a_Rot)
            self.Done = terminated 

            # 3) CALC REWARD
            try:
                reward = self._CalcReward()  
            except (UnboundLocalError,ValueError) as e:
                reward = np.nan
                print(RED + f"Error: {e}" + RESET)

            info_dict = {
                "reward_vals": self.reward_vals,
                "reward": reward,
                "a_Rot": a_Rot,
                "Trg_Flag": self.Trg_Flag,
                "Impact_Flag_Ext": self.Impact_Flag_Ext,
                "D_perp_pad_min": self.D_perp_pad_min,
                "Tau_CR_trg": self.Tau_CR_trg,
                "Plane_Angle": self.Plane_Angle_deg,
                "V_mag": self.V_mag,
                "V_angle": self.V_angle,
                "TestCondition_idx": self.TestCondition_idx,
            }


            # 5) RETURN VALUES
            return(
                self.obs_trg,
                reward,
                terminated,
                truncated,
                info_dict,
            )
        
    def _finishSim(self,a_Rot):
        
        OnceFlag_Trg = False
        OnceFlag_Impact = False

        terminated = False
        truncated = False

        ## SEND TRIGGER ACTION TO CONTROLLER
        self.sendCmd("Policy",[0.0,a_Rot,self.Ang_Acc_range[0]],cmd_flag=self.Ang_Acc_range[1])
        print("Policy is sent", self.Ang_Acc_range[1])

        ## RUN REMAINING STEPS AT FULL SPEED
        self.pausePhysics(False)

        while not (terminated or truncated):

            t_now = self._getTime()

            ## START TRIGGER AND IMPACT TERMINATION TIMERS
            if (self.Trg_Flag == True and OnceFlag_Trg == False):
                self.start_time_trg = t_now     # Starts countdown for when to reset run
                OnceFlag_Trg = True             # Turns on to make sure this only runs once per rollout

            if (self.Impact_Flag_Ext and OnceFlag_Impact == False):
                self.start_time_trg = np.nan    # Nullify trigger timer
                self.start_time_impact = t_now
                OnceFlag_Impact = True


            # 4) CHECK TERMINATION/TRUNCATED
            r_B_O =  self.r_B_O
            V_B_O = self.V_B_O
            r_P_B = self.R_WP(self.r_P_O - r_B_O,self.Plane_Angle_rad) # {t_x,n_p}
            V_B_P = self.R_WP(V_B_O,self.Plane_Angle_rad) # {t_x,n_p}

            # ============================
            ##    Termination Criteria 
            # ============================

            if self.Done == True:
                self.error_str = "Episode Completed: Done [Terminated] "
                terminated = True
                truncated = False
                # print(YELLOW,self.error_str,RESET)

            ## TRIGGER TIMEOUT  
            elif (t_now - self.start_time_trg) > self.t_trg_max:
                self.error_str = "Episode Completed: Pitch Timeout [Truncated] "
                terminated = False
                truncated = True
                # print(YELLOW,self.error_str,f"{(t_now - self.start_time_trg):.3f} s",RESET)

            ## IMPACT TIMEOUT
            elif (t_now - self.start_time_impact) > self.t_impact_max:
                self.error_str = "Episode Completed: Impact Timeout [Truncated] "
                terminated = False
                truncated = True
                # print(YELLOW,self.error_str,f"{(t_now - self.start_time_impact):.3f} s",RESET)

            elif r_B_O[2] < -15:
                self.error_str = "Episode Completed: Out of bounds [Terminated]"
                terminated = True
                truncated = False
                # print(YELLOW,self.error_str,RESET)

            elif np.abs(r_P_B[0]) > 1.4 and (self.D_perp < 1.5*self.L_eff) and (self.D_perp >= 0):
                self.error_str = "Episode Completed: Out of Bounds [Terminated]"
                terminated = True
                truncated = False
                # print(YELLOW,self.error_str,RESET)

            ## REAL-TIME TIMEOUT
            elif (time.time() - self.start_time_real) > self.t_real_max:
                self.error_str = "Episode Completed: Episode Time Exceeded [Truncated] "
                terminated = False
                truncated = True
                # print(YELLOW,self.error_str,f"{(time.time() - self.start_time_real):.3f} s",RESET)

            else:
                terminated = False
                truncated = False

        return terminated,truncated

    def _CalcReward(self):
        if self.Impact_Flag_Ext:
            V_B_O_impact = self.R_PW(self.V_B_P_impact_Ext,self.Plane_Angle_rad)    # {X_W,Y_W,Z_W}
            V_hat_impact = V_B_O_impact/np.linalg.norm(V_B_O_impact)                # {X_W,Y_W,Z_W}

    def render(self):
        ## DO NOTHING ##
        return

    def close(self):
        ## DO NOTHING ##
        return

if __name__ == "__main__":

    rclpy.init()

    env = SAR_Sim_DeepRL(Ang_Acc_range=[-90,0],V_mag_range=[1.5,3.5],V_angle_range=[5,175],Plane_Angle_range=[0,180],Render=True,Fine_Tune=False)

    time.sleep(3)
    env._setTestingConditions()
    env._initialStep()
    print("_initialStep is done")
    action = env.action_space.sample()
    action[0] = 0.0
    action[1] = -1.0
    env.step(action)
    print("env.step(action) is done")

    rclpy.spin(env.node)

    #rclpy.shutdown()