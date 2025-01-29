## IMPORT ENVIRONMENTS
from RL_Manager_DH import RL_Training_Manager
from Envs.SAR_Sim_DeepRL_DH import SAR_Sim_DeepRL

## STANDARD IMPORTS
import os
from datetime import datetime
import rospkg
import argparse
import json
import rclpy

# For run this code I need to command below in terminal.
# python3 T3_Policy_Data_Collection_DH.py --TrainConfig /home/dlee/ros2_ws/src/sar_simulation/sar_projects/NewDeepRL/Config_Files/SOV5_3D_Sim/Data_SOV5_A30_L200_0deg_aRot90_S3D_DH.json

## DEFINE BASE PATH
workspace_path = os.path.expanduser('~/ros2_ws')
BASE_PATH = os.path.join(workspace_path, 'src', 'sar_simulation')
LOG_DIR = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs" 

## ARGUMENT PARSER
parser = argparse.ArgumentParser(description='Policy Pre-Training Script')
parser.add_argument('--GroupName',      help='Log group name', default='')
parser.add_argument('--TrainConfig',    help='Path to training config file', required=True)
parser.add_argument('--t_step_load',    help='Time step to load model', default=None, type=int)
parser.add_argument('--S3_Upload',      help='Upload to S3', default=False, type=bool)
args = parser.parse_args()


## LOAD TRAINING MODEL CONFIGURATION
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

    ## LOGGING CONFIGURATION
    LogName = TrainConfig['LogName']
    PlaneAngle = TrainConfig['ENV_KWARGS']['Plane_Angle_range'][0]
    if args.t_step_load == None:
        t_step_load = TrainConfig['t_step_optim']
    else:
        t_step_load = args.t_step_load

    
    ## SELECT ENVIRONMENT
    if TrainConfig['ENV_Type'] == "SAR_2D_Env":
        env = SAR_Sim_DeepRL
    elif TrainConfig['ENV_Type'] == "SAR_Sim_DeepRL":
        env = SAR_Sim_DeepRL

    ## SET UP TRAINING CONDITIONS FROM CONFIG
    env_kwargs = TrainConfig['ENV_KWARGS']

    ## CREATE RL MANAGER
    RL_Manager = RL_Training_Manager(env,args.GroupName,TrainConfig['LogName'],env_kwargs=env_kwargs,S3_Upload=args.S3_Upload)
    RL_Manager.env.Fine_Tuning_Flag = False
    
    ## CREATE MODEL AND TRAIN
    RL_Manager.create_model()
    RL_Manager.load_model(
        GroupName=args.GroupName,
        LogName=TrainConfig['LogName'],
        t_step_load=t_step_load,
        Params_only=True,
    )

    RL_Manager.collect_landing_performance(
        Plane_Angle_Step=45,
        V_mag_Step=0.5,
        V_angle_Step=5,
        n=5,
    )
    
    # for i in [0,45,90,135,180]:
    #     try:
    #         RL_Manager.plot_landing_performance(PlaneAngle=i,saveFig=True,showFig=False)
    #     except:
    #         pass
    # RL_Manager.save_NN_to_C_header()
