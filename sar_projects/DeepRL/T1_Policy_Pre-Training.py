## IMPORT ENVIRONMENTS
from RL_Manager import RL_Training_Manager
from Envs.SAR_Sim_DeepRL import SAR_Sim_DeepRL

## STANDARD IMPORTS
import os
from datetime import datetime
import argparse
import json
import rclpy

## DEFINE BASE PATH
workspace_path = os.path.expanduser('~/ros2_ws')
BASE_PATH = os.path.join(workspace_path, 'src', 'sar_simulation')

# python3 T1_Policy_Pre-Training.py --TrainConfig /home/dlee/ros2_ws/src/sar_simulation/sar_projects/DeepRL/Config_Files/SOV5_3D_Sim/SOV5_A30_L200_0deg_aRot90_S3D.json
## ARGUMENT PARSER
parser = argparse.ArgumentParser(description='Policy Pre-Training Script')
parser.add_argument('--TrainConfig',    help='Path to training config file', required=True)
parser.add_argument('--GroupName',      help='Log group name', default='')
parser.add_argument('--S3_Upload',      help='Upload to S3', default=False, type=bool)
args = parser.parse_args()

## LOAD CONFIGURATION
with open(args.TrainConfig, 'r') as TrainConfig_File:
    TrainConfig = json.load(TrainConfig_File)


## UPDATE SAR TYPE AND SAR CONFIG IN BASE SETTINGS FILE
Base_Settings_yaml = f"{BASE_PATH}/sar_config/Base_Settings.yaml"
with open(Base_Settings_yaml, 'r') as file:
    lines = file.readlines()

for i, line in enumerate(lines):
    if line.strip().startswith('SAR_Type:'):
        lines[i] = f"  SAR_Type: '{TrainConfig['SAR_SETTINGS']['SAR_Type']}'\n"
    elif line.strip().startswith('SAR_Config:'):
        lines[i] = f"  SAR_Config: '{TrainConfig['SAR_SETTINGS']['SAR_Config']}'\n"

with open(Base_Settings_yaml, 'w') as file:
    file.writelines(lines)


if __name__ == '__main__':

    rclpy.init()

    ## SELECT ENVIRONMENT
    if TrainConfig['ENV_Type'] == "SAR_2D_Env":
        env = SAR_Sim_DeepRL
    elif TrainConfig['ENV_Type'] == "SAR_Sim_DeepRL":
        env = SAR_Sim_DeepRL

    ## SET UP TRAINING CONDITIONS FROM CONFIG
    env_kwargs = TrainConfig['ENV_KWARGS']

    ## CREATE RL MANAGER
    RL_Manager = RL_Training_Manager(env,args.GroupName,TrainConfig['LogName'],env_kwargs=env_kwargs,S3_Upload=args.S3_Upload)

    ## CREATE MODEL AND TRAIN
    RL_Manager.create_model()
    RL_Manager.train_model(reset_timesteps=False,t_step_max=TrainConfig['t_step_limit'])

    rclpy.shutdown()