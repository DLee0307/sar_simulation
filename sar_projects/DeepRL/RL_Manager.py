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

# import rospy
# import rospkg
# import glob
# import boto3

from collections import deque

# import matplotlib.pyplot as plt
# import matplotlib as mpl
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
      print()

    def train_model(self,model_save_freq=2e3,reward_check_freq=500,S3_upload_freq=1000,reset_timesteps=False,t_step_max=500e3):
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
    def __init__(self, RLM, 
                 reward_check_freq: int = 500, 
                 S3_upload_freq: int = 500,
                 model_save_freq: int = 5_000, 
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
        

    def _on_step(self) -> bool:
        
        ep_info_buffer = self.locals['self'].ep_info_buffer


        print("RL_Manager RewardCallback class's _on_step function is done")