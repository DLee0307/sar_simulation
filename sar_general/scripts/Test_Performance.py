#!/usr/bin/env python3

import threading
import os
import sys
import rclpy
from rclpy.node import Node
import numpy as np
import csv

# SET PYTHONPATH
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.append('/home/dlee/ros2_ws/src/sar_simulation')

from sar_env import SAR_Sim_Interface


YELLOW = '\033[93m'
RED = '\033[91m'
GREEN = '\033[92m'
BLUE = "\033[34m"  
RESET = "\033[0m"  # Reset to default color

        
def main(args=None):
    rclpy.init()
    
    ## INIT GAZEBO ENVIRONMENT
    env = SAR_Sim_Interface(GZ_Timeout=False)
    env.pausePhysics(False)
    #env.adjustSimSpeed(0.7)

    ## DEFINE BASE PATH
    workspace_path = os.path.expanduser('~/ros2_ws')
    BASE_PATH = os.path.join(workspace_path, 'src', 'sar_simulation')
    LOG_DIR = f"{BASE_PATH}/sar_general" 

    fileName = "PolicyPerformance_Data.csv"
    filePath = os.path.join(LOG_DIR,fileName)

    Plane_Angle_range0 = 0
    Plane_Angle_range1 = 0
    Plane_Angle_Step = 45
    
    V_mag_range0 = 1
    V_mag_range1 = 4
    V_mag_Step = 0.5

    V_angle_range0 = 75
    V_angle_range1 = 90
    V_angle_Step = 5

    ## GENERATE SWEEP ARRAYS
    Plane_Angle_num = np.ceil((Plane_Angle_range1 - Plane_Angle_range0) / Plane_Angle_Step).astype(int) + 1
    Plane_Angle_arr = np.linspace(Plane_Angle_range0, Plane_Angle_range1, Plane_Angle_num, endpoint=True)

    V_mag_num = np.ceil((V_mag_range1 - V_mag_range0) / V_mag_Step).astype(int) + 1
    V_mag_arr = np.linspace(V_mag_range0, V_mag_range1, V_mag_num, endpoint=True)

    V_angle_num = np.ceil((V_angle_range1 - V_angle_range0) / V_angle_Step).astype(int) + 1
    V_angle_arr = np.linspace(V_angle_range0, V_angle_range1, V_angle_num, endpoint=True)

    n = 1
    num_trials = len(V_mag_arr)*len(V_angle_arr)*len(Plane_Angle_arr)*n
    idx = 0

    with open(filePath,'w') as file:
                writer = csv.writer(file,delimiter=',')
                writer.writerow([
                    "V_mag", "V_angle", "Plane_Angle", "Trial_num",

                    "--",

                    "Pad_Connections",
                    "BodyContact","ForelegContact","HindlegContact",

                    "--",
                    
                    "a_Trg_trg",
                    "a_Rot_trg",
                    "Vel_mag_B_O_trg","Vel_angle_B_O_trg",
                    "Vel_mag_B_P_trg","Vel_angle_B_P_trg",

                    "Tau_CR_trg",
                    "Tau_trg",
                    "Theta_x_trg",
                    "D_perp_CR_trg",
                    "D_perp_trg",

                    "--",

                    "Phi_B_O_impact",
                    "Phi_B_P_impact",
                    "Omega_B_O_impact",

                    "Vel_B_P_impact_x","Vel_B_P_impact_z",
                    
                    "Impact_Magnitude",
                    "Force_Impact_x","Force_Impact_y","Force_Impact_z",

                    "--",

                    "reward","reward_vals",
                    "NN_Output_trg","a_Rot_scale",

                    "--",

                    "4_Leg_NBC","4_Leg_BC",
                    "2_Leg_NBC","2_Leg_BC",
                    "0_Leg_NBC","0_Leg_BC",
                ])    

    for V_mag in V_mag_arr:
        for V_angle in V_angle_arr:
            for trial in range(n):

    rclpy.shutdown()
if __name__ == '__main__':
    main()
