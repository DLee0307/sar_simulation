## STANDARD IMPORTS
import sys
import os
from ament_index_python.packages import get_package_share_directory

from datetime import datetime,timedelta
import numpy as np
import pandas as pd
import torch as th
import yaml
import csv
import time 


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

