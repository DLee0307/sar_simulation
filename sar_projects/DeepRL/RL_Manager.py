## STANDARD IMPORTS
import sys
import os

from datetime import datetime,timedelta
import numpy as np
import pandas as pd
import torch as th
import yaml
import csv
import time 
import glob

# import rospy
# import rospkg
# import boto3

from collections import deque

import matplotlib.pyplot as plt
import matplotlib as mpl
# from scipy.interpolate import griddata

## SB3 Imports
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import *
from stable_baselines3.common import utils
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.utils import safe_mean
from stable_baselines3.common.logger import Figure

## DEFINE BASE PATH
workspace_path = os.path.expanduser('~/ros2_ws')
BASE_PATH = os.path.join(workspace_path, 'src', 'sar_simulation')
LOG_DIR = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs" 
#print("LOG_DIR: ", LOG_DIR)

## COLLECT CURRENT TIME
current_datetime = datetime.now()
current_time = current_datetime.strftime("%m_%d-%H:%M")
#print("current_time: ", current_time)

class RL_Training_Manager():

    def __init__(self,env,Group_Name,Log_Name,env_kwargs=None,S3_Upload=False):

        ## SET UP ENVIRONMENT
        self.vec_env = make_vec_env(env, env_kwargs=env_kwargs)
        self.env = self.vec_env.envs[0].unwrapped

        ## SET UP S3 CONNECTIONS
        # !!! need to change
        # self.S3_client = boto3.client('s3')
        self.S3_Upload_Flag = S3_Upload

        self.Group_Name = Group_Name
        self.Log_Name = Log_Name

        self.LogGroup_Dir = os.path.join(LOG_DIR,Group_Name)
        self.Log_Dir = os.path.join(LOG_DIR,Group_Name,Log_Name)
        self.Model_Dir = os.path.join(self.Log_Dir,"Models")
        self.TB_Log_Dir = os.path.join(self.Log_Dir,"TB_Logs")

        os.makedirs(self.Log_Dir, exist_ok=True)
        os.makedirs(self.Model_Dir, exist_ok=True)
        os.makedirs(self.TB_Log_Dir, exist_ok=True)

    def create_model(self,model_kwargs=None,write_config=True):

        if model_kwargs is None:
            model_kwargs = {
                "gamma": 0.999,
                "learning_rate": 1.5e-3,
                "net_arch": dict(pi=[10,10,10], qf=[64,64,64]),
                "ent_coef": "auto_0.05",
                "target_entropy": -2,
                "batch_size": 256,
                "buffer_size": int(200e3),
            }

        self.model = SAC(
            "MlpPolicy",
            env=self.vec_env,
            gamma=model_kwargs["gamma"],
            learning_rate=model_kwargs["learning_rate"],
            ent_coef=model_kwargs["ent_coef"],
            target_entropy=model_kwargs["target_entropy"],
            buffer_size=model_kwargs["buffer_size"],
            policy_kwargs=dict(activation_fn=th.nn.LeakyReLU,net_arch=model_kwargs["net_arch"]),
            replay_buffer_kwargs=dict(handle_timeout_termination=False),
            learning_starts=0,
            train_freq=(1,'episode'),
            gradient_steps=-1,
            verbose=1,
            device='cpu',
            tensorboard_log=self.TB_Log_Dir
        )

        if write_config:
            self.write_config_file()

        #print("RL_Manager create_model function is done")

    def load_model(self,t_step_load: int, GroupName=None, LogName=None, Params_only=False, load_replay_buffer=False):
        
        if GroupName is None:
            GroupName = self.Group_Name
        
        if LogName is None:
            LogName = self.Log_Name

        ## SEARCH FOR BOTH MODEL AND REPLAY BUFFER WITH WILDCARD FOR OPTIONAL SUFFIX
        model_pattern = f"model_{int(t_step_load)}_steps*.zip" 
        replay_buffer_pattern = f"replay_buffer_{int(t_step_load)}_steps*.pkl" 

        ## FIND MODEL AND REPLAY BUFFER FILES
        model_files_path = os.path.join(LOG_DIR,GroupName,LogName,"Models", model_pattern)
        print(f"Loading Model params from model: {model_files_path}")
        model_files = glob.glob(os.path.join(LOG_DIR,GroupName,LogName,"Models", model_pattern))
        replay_buffer_files = glob.glob(os.path.join(LOG_DIR,GroupName,LogName, "Models", replay_buffer_pattern))

        ## LOAD MODEL AND REPLAY BUFFER
        if Params_only:
            model_path = model_files[0]  # Taking the first match
            self.model.set_parameters(model_path,exact_match=False,device='cpu')
            # loaded_model = SAC.load(
            #     model_path,
            #     device='cpu',
            # )
            # self.model.policy.load_state_dict(loaded_model.policy.state_dict())
        
        else:
            print(f"Loading Model...")
            model_path = model_files[0]  # Taking the first match
            self.model = SAC.load(
                model_path,
                env=self.vec_env,
                device='cpu',
            )

        if load_replay_buffer:

            print(f"Loading Replay Buffer...")
            replay_buffer_path = replay_buffer_files[0]  # Similarly, taking the first match
            self.model.load_replay_buffer(replay_buffer_path)

    # model_save_freq=2e3
    def train_model(self,model_save_freq=100,reward_check_freq=500,S3_upload_freq=1000,reset_timesteps=False,t_step_max=500e3):
        if reset_timesteps == True:

            self.model.tensorboard_log = self.TB_Log_Dir
            self.model.learning_starts = 0

        reward_callback = RewardCallback(self,
                                         model_save_freq=model_save_freq,
                                         S3_upload_freq=S3_upload_freq,
                                         reward_check_freq=reward_check_freq
                                         )
        #!! DH added for debugging there is an error for initializing self.locals
        #reward_callback._on_step()
        #print(self.locals)

        self.model.learn(
            total_timesteps=int(t_step_max),
            callback=reward_callback,
            tb_log_name="TB_Log",
            reset_num_timesteps=reset_timesteps,
        )

        print("RL_Manager train_model function is done")

    def test_policy(self,V_mag=None,V_angle=None,Plane_Angle=None):

        if V_mag != None:
            self.env.V_mag_range = [V_mag,V_mag]

        if V_angle != None:
            self.env.V_angle_range = [V_angle,V_angle]

        if Plane_Angle != None:
            self.env.Plane_Angle_range = [Plane_Angle,Plane_Angle]

        obs,_ = self.env.reset()
        terminated = False
        truncated = False
 
        while not (terminated or truncated):
            action,_ = self.model.predict(obs)
            obs,reward,terminated,truncated,_ = self.env.step(action)

        return obs,reward

    def sweep_policy(self,Plane_Angle_Step=45,V_mag_Step=0.5,V_angle_Step=10,n=1):
        print()

    def collect_landing_performance(self,fileName=None,Plane_Angle_Step=45,V_mag_Step=0.5,V_angle_Step=10,n=1):
        print()

    def plot_landing_performance(self,PlaneAngle=0,fileName=None,saveFig=False,showFig=True):
        print()

    def save_NN_to_C_header(self):
        print()

    def write_config_file(self):
        config_path = os.path.join(self.Log_Dir,"Config.yaml")

        General_Dict = dict(

            SAR_SETTINGS = dict(
                SAR_Type = self.env.SAR_Type,
                SAR_Config = self.env.SAR_Config,
            ),

            PLANE_SETTINGS = dict(
                Plane_Type = self.env.Plane_Type,
                Plane_Config = self.env.Plane_Config,
            ),

            ENV_SETTINGS = dict(
                Env_Name = self.env.Env_Name,
                V_mag_Limts = self.env.V_mag_range,
                V_angle_Limits = self.env.V_angle_range,
                Plane_Angle_Limits = self.env.Plane_Angle_range,
                Ang_Acc_Limits = self.env.Ang_Acc_range,
            ),

            MODEL_SETTINGS = dict(
                Mass_Std = self.env.Mass_std,
                Iyy_Std = self.env.Iyy_std,
                Ref_Mass = self.env.Ref_Mass,
                Ref_Ixx = self.env.Ref_Ixx,
                Ref_Iyy = self.env.Ref_Iyy,
                Ref_Izz = self.env.Ref_Izz,
                L_eff = self.env.L_eff,
                Gamma_eff = float(self.env.Gamma_eff),
                Forward_Reach = self.env.Forward_Reach,
            ),

            LEARNING_MODEL = dict(
                Policy = self.model.policy.__class__.__name__,
                Observation_Layer = self.model.policy.observation_space.shape[0],
                Network_Layers = self.model.policy.net_arch,
                Action_Layer = self.model.policy.action_space.shape[0]*2,
                Action_Space_High = self.model.policy.action_space.high.tolist(),
                Action_Space_Low = self.model.policy.action_space.low.tolist(),
                Gamma = self.model.gamma,
                Learning_Rate = self.model.learning_rate,
            ),

            REWARD_SETTINGS=self.env.reward_weights
        )

        with open(config_path, 'w') as outfile:
            yaml.dump(General_Dict,outfile,default_flow_style=False,sort_keys=False)

        #print("RL_Manager write_config_file function is done")

    def load_config_file(self,config_path): ##DH cannot find where this function is used.

        with open(config_path, 'r') as file:
            config_dict = yaml.safe_load(file)

        self.env.setAngAcc_range(config_dict['ENV_SETTINGS']['Ang_Acc_Limits'])

class RewardCallback(BaseCallback):
    # model_save_freq: int = 5_000
    def __init__(self, RLM, 
                 reward_check_freq: int = 500, 
                 S3_upload_freq: int = 500,
                 model_save_freq: int = 100, 
                 keep_last_n_models: int = 5,
                 verbose=0):
        super(RewardCallback, self).__init__(verbose)

        ## RL MANAGER
        self.RLM = RLM

        ## CALLBACK FREQUENCIES
        self.reward_check_freq = reward_check_freq
        self.S3_upload_freq = S3_upload_freq
        self.model_save_freq = model_save_freq


        ## MODEL SAVING
        self.saved_models = []
        self.keep_last_n_models = keep_last_n_models

        ## REWARD TRACKING
        self.best_mean_reward = -np.inf
        self.rew_mean_window = deque(maxlen=int(15e3))

        ## REWARD STABILITY
        self.rew_mean_diff_threshold = 0.12
        self.ent_burst_cooldown = int(30e3)                 # Cooldown period for entropy burst
        self.ent_burst_limit = int(100e3)                    # Limit the entropy burst to the first 100k steps
        self.last_ent_burst_step = 0                        # Last step when entropy burst was applied

        ## REWARD GRID
        self.flight_tuple_list = []
        self.last_processed_idx = 0

        self.Vx_bins = np.arange(0,5,step=0.25)
        self.Vz_bins = np.arange(-5.0,5,step=0.25)
        self.reward_grid = np.full((len(self.Vx_bins), len(self.Vz_bins)), np.nan)

        print("RL_Manager RewardCallback initializion is done")

    def _on_training_start(self) -> None:
        """
        This method is called before the first rollout starts.
        """
        self.env = self.training_env.envs[0].unwrapped

        ## GRAB TB LOG FILE
        TB_Log_pattern = f"TB_Log_*"
        TB_Log_folders = glob.glob(os.path.join(self.RLM.TB_Log_Dir, TB_Log_pattern))
        self.TB_Log = os.listdir(TB_Log_folders[0])[0]
        self.TB_Log_file_path = os.path.join(TB_Log_folders[0],self.TB_Log)


    def _on_step(self) -> bool:

        ep_info_buffer = self.locals['self'].ep_info_buffer

        ## COMPUTE EPISODE REWARD MEAN, VARIANCE, AND MAX-MIN DIFFERENCE
        ep_rew_mean = safe_mean([ep_info["r"] for ep_info in ep_info_buffer])
        self.rew_mean_window.append(ep_rew_mean)
        ep_rew_mean_max = np.nanmax(self.rew_mean_window)
        ep_rew_mean_min = np.nanmin(self.rew_mean_window)
        ep_rew_mean_var = np.var(self.rew_mean_window)
        ep_rew_mean_diff = ep_rew_mean_max - ep_rew_mean_min

        ## LOG VARIANCE HISTORY
        self.logger.record("rollout/ep_rew_mean_var", ep_rew_mean_var)
        self.logger.record("rollout/ep_rew_mean_diff", ep_rew_mean_diff)

        ## GIVE ENTROPY KICK BASED ON REWARD STABILITY
        self._give_entropy_burst(ep_rew_mean,ep_rew_mean_diff)

        if self.num_timesteps % 5000 == 0:
            self._save_reward_grid_plot_to_TB()

        ## SAVE REWARD GRID PLOT AND MODEL EVERY N TIMESTEPS
        if self.num_timesteps % self.model_save_freq == 0:
            self._save_model_and_replay_buffer()

        ## CHECK FOR MODEL PERFORMANCE AND SAVE IF IMPROVED
        if self.num_timesteps % self.reward_check_freq == 0 and self.env.K_ep > 2.5*len(self.env.TestingConditions):

            ## COMPUTE THE MEAN REWARD FOR THE LAST 'CHECK_FREQ' EPISODES
            if ep_rew_mean > self.best_mean_reward:
                
                ## SAVE BEST MODEL AND REPLAY BUFFER
                self.best_mean_reward = ep_rew_mean
                self._save_best_model_and_replay_buffer()

                ## REMOVE OLDER MODELS AND REPLAY BUFFERS
                self._keep_last_n_models()


        ## CHECK IF EPISODE IS DONE
        if self.locals["dones"].item():

            info_dict = self.locals["infos"][0]

            ## CONVERT PLANE RELATIVE ANGLES TO GLOBAL ANGLE
            V_angle_global = info_dict["V_angle"] - info_dict["Plane_Angle"]
            V_mag = info_dict["V_mag"]

            ## SAVE FLIGHT TUPLE TO LIST
            Vx = V_mag*np.cos(np.radians(V_angle_global))
            Vz = V_mag*np.sin(np.radians(V_angle_global))
            reward = self.locals["rewards"].item()
            self.flight_tuple_list.append((Vx,Vz,reward))

            ## TB LOGGING VALUES
            self.logger.record('Custom/K_ep',self.env.K_ep)
            self.logger.record('Custom/Reward',self.locals["rewards"].item())
            self.logger.record('Custom/LogName',self.RLM.Log_Name)

            self.logger.record('z_Custom/Vel_mag',info_dict["V_mag"])
            self.logger.record('z_Custom/Vel_angle',info_dict["V_angle"])
            self.logger.record('z_Custom/Plane_Angle',info_dict["Plane_Angle"])
            self.logger.record('z_Custom/a_Rot_trg',info_dict["a_Rot"])
            self.logger.record('z_Custom/Tau_CR_trg',info_dict["Tau_CR_trg"])
            self.logger.record('z_Custom/Trg_Flag',int(info_dict["Trg_Flag"]))
            self.logger.record('z_Custom/Impact_Flag_Ext',int(info_dict["Impact_Flag_Ext"]))
            self.logger.record('z_Custom/D_perp_pad_min',info_dict["D_perp_pad_min"])
            self.logger.record('z_Custom/TestCondition_idx',info_dict["TestCondition_idx"])

            self.logger.record('z_Rewards_Components/R_Dist',info_dict["reward_vals"][0])
            self.logger.record('z_Rewards_Components/R_tau',info_dict["reward_vals"][1])
            self.logger.record('z_Rewards_Components/R_tx',info_dict["reward_vals"][2])
            self.logger.record('z_Rewards_Components/R_LT',info_dict["reward_vals"][3])
            self.logger.record('z_Rewards_Components/R_GM',info_dict["reward_vals"][4])
            self.logger.record('z_Rewards_Components/R_phi',info_dict["reward_vals"][5])
            self.logger.record('z_Rewards_Components/R_Legs',info_dict["reward_vals"][6])


        ## UPLOAD TB LOG TO SB3
        if self.num_timesteps % self.S3_upload_freq == 0:
            print()
            # print("self.num_timesteps", self.num_timesteps)
            # print("self.S3_upload_freq",self.S3_upload_freq)

            # self.RLM.upload_file_to_S3(local_file_path=self.TB_Log_file_path,S3_file_path=os.path.join("S3_TB_Logs",self.RLM.Group_Name,self.RLM.Log_Name,"TB_Logs/TB_Log_0",self.TB_Log))

        #print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
        print("RL_Manager RewardCallback class's _on_step function is done")

        return True

    def _give_entropy_burst(self,ep_rew_mean,ep_rew_mean_diff):
        '''
        Give an entropy burst to the model if the reward mean is stable and above a certain threshold.
        '''

        if (ep_rew_mean_diff < self.rew_mean_diff_threshold
            and ep_rew_mean > 0.3 
            and self.num_timesteps > self.last_ent_burst_step + self.ent_burst_cooldown
            and int(20e3) < self.num_timesteps < self.ent_burst_limit):

            self.last_ent_burst_step = self.num_timesteps # Update last entropy burst step

            ## APPLY ENTROPY BURST
            with th.no_grad():
                ent_coef = 0.03
                self.model.log_ent_coef.fill_(np.log(ent_coef))

        ## SET ENTROPY COEFFICIENT TO ZERO AFTER BURST LIMIT
        elif self.ent_burst_limit <= self.num_timesteps:

            ## TURN OFF ENTROPY COEFFICIENT
            with th.no_grad():
                self.model.ent_coef_optimizer = None
                ent_coef = 0.00
                self.model.ent_coef_tensor = th.tensor(float(ent_coef), device=self.model.device)

    # Need to check if this function works well
    def _save_reward_grid_plot_to_TB(self):
        '''
        Save the reward grid plot to tensorboard.
        '''

        ## PROCESS FLIGHT TUPLE LIST WITH NEWEST DATA
        for Vx, Vz, reward in self.flight_tuple_list[self.last_processed_idx:]:
            Vx_idx = np.digitize(Vx, self.Vx_bins) - 1
            Vz_idx = np.digitize(Vz, self.Vz_bins) - 1
            self.last_processed_idx += 1

            if 0 <= Vx_idx < len(self.Vx_bins)-1 and 0 <= Vz_idx < len(self.Vz_bins)-1:
                self.reward_grid[Vx_idx, Vz_idx] = reward

        ## PLOT REWARD GRID
        fig = plt.figure()
        ax = fig.add_subplot()
        cmap = plt.cm.jet
        norm = mpl.colors.Normalize(vmin=0,vmax=1)

        heatmap = ax.imshow(self.reward_grid.T, 
                    interpolation='nearest', 
                    cmap=cmap, 
                    extent=[0.0,5.0,-5.0,5.0], 
                    aspect='equal', 
                    origin='lower',
                    zorder=0,
                    norm=norm)

        ax.set_xlim(0.0, 5.0)
        ax.set_ylim(-5.0, 5.0)

        ax.set_xlabel('Vx (m/s)')
        ax.set_ylabel('Vz (m/s)')
        ax.set_title('Reward Grid')
        
        # SAVE AND DUMP LOG TO TB FILE
        self.logger.record("LandingSuccess/figure", Figure(fig, close=True), exclude=("stdout", "log", "json", "csv"))
        self.logger.dump(self.num_timesteps)
        plt.close()

        print("_save_reward_grid_plot_to_TB function is done")

    # Need to check if this function works well
    def _save_model_and_replay_buffer(self):
            
        ## SAVE NEWEST MODEL AND REPLAY BUFFER
        model_name = f"model_{self.num_timesteps}_steps.zip"
        model_path = os.path.join(self.RLM.Model_Dir,model_name)

        replay_buffer_name = f"replay_buffer_{self.num_timesteps}_steps.pkl"
        replay_buffer_path = os.path.join(self.RLM.Model_Dir, replay_buffer_name)

        self.model.save(model_path)
        self.model.save_replay_buffer(replay_buffer_path)

        # ## UPLOAD MODEL AND REPLAY BUFFER TO S3
        # self.RLM.upload_file_to_S3(local_file_path=model_path, S3_file_path=os.path.join("S3_TB_Logs",self.RLM.Group_Name,self.RLM.Log_Name,"Models",model_name))
        # self.RLM.upload_file_to_S3(local_file_path=replay_buffer_path, S3_file_path=os.path.join("S3_TB_Logs",self.RLM.Group_Name,self.RLM.Log_Name,"Models",replay_buffer_name))

        print("_save_model_and_replay_buffer function is done")

    # Need to check if this function works well
    def _save_best_model_and_replay_buffer(self):

        model_name = f"model_{self.num_timesteps}_steps_BestModel.zip"
        model_path = os.path.join(self.RLM.Model_Dir, model_name)
        s3_model_path = os.path.join("S3_TB_Logs", self.RLM.Group_Name, self.RLM.Log_Name, "Models", model_name)

        replay_buffer_name = f"replay_buffer_{self.num_timesteps}_steps_BestModel.pkl"
        replay_buffer_path = os.path.join(self.RLM.Model_Dir, replay_buffer_name)
        s3_replay_buffer_path = os.path.join("S3_TB_Logs", self.RLM.Group_Name, self.RLM.Log_Name, "Models", replay_buffer_name)

        ## SAVE MODEL AND REPLAY BUFFER
        self.model.save(model_path)
        self.model.save_replay_buffer(replay_buffer_path)

        ## UPLOAD MODEL AND REPLAY BUFFER TO S3
        # self.RLM.upload_file_to_S3(local_file_path=model_path, S3_file_path=os.path.join("S3_TB_Logs",self.RLM.Group_Name,self.RLM.Log_Name,"Models",model_name))
        # self.RLM.upload_file_to_S3(local_file_path=replay_buffer_path, S3_file_path=os.path.join("S3_TB_Logs",self.RLM.Group_Name,self.RLM.Log_Name,"Models",replay_buffer_name))


        if self.verbose > 0:
            print(f"New best mean reward: {self.best_mean_reward:.2f} - Model and replay buffer saved.")

        ## TRACK SAVED MODELS FOR DELETION
        self.saved_models.append((model_path, replay_buffer_path, s3_model_path, s3_replay_buffer_path))

    # Need to check if this function works well
    def _keep_last_n_models(self):

        ## REMOVE OLDER TOP MODELS AND REPLAY BUFFERS
        if len(self.saved_models) > self.keep_last_n_models:

            for model_path, replay_buffer_path, s3_model_path, s3_replay_buffer_path in self.saved_models[:-self.keep_last_n_models]:
                
                # Delete local files
                if os.path.exists(model_path):
                    os.remove(model_path)
                if os.path.exists(replay_buffer_path):
                    os.remove(replay_buffer_path)
                
                # Delete from S3
                self.RLM.delete_file_from_S3(s3_model_path)
                self.RLM.delete_file_from_S3(s3_replay_buffer_path)

            self.saved_models = self.saved_models[-self.keep_last_n_models:]