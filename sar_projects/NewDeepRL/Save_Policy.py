## IMPORT ENVIRONMENTS
from RL_Manager_DH import RL_Training_Manager
from Envs.SAR_Sim_DeepRL_DH import SAR_Sim_DeepRL

## STANDARD IMPORTS
import os
from datetime import datetime
import argparse
import json
import rclpy

# For run this code I need to command below in terminal.
# python3 Save_Policy.py 



if __name__ == '__main__':

    rclpy.init()

    # 환경 설정
    env_kwargs = {
        "Ang_Acc_range": [-90.0, -80.0],
        "Plane_Angle_range": [0, 0],
        "V_mag_range": [1.0, 4.0],
        "V_angle_range": [75, 90],
        "Render": False,
        "GZ_Timeout": True
    }
    RL_Manager = RL_Training_Manager(
        env=SAR_Sim_DeepRL, 
        Group_Name='', 
        Log_Name='SOV5_A30_L200_0deg_aRot90_S3D', 
        env_kwargs=env_kwargs
    )

    # 저장된 모델 및 리플레이 버퍼 로드
    RL_Manager.load_model(
        t_step_load=30000, 
        GroupName='', 
        LogName='SOV5_A30_L200_0deg_aRot90_S3D', 
        Params_only=False, 
        load_replay_buffer=True
    )

    RL_Manager.save_NN_to_C_header()


    rclpy.shutdown()
    
