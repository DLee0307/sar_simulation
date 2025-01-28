#!/usr/bin/env python3

import threading
import os
import sys
import rclpy
from rclpy.node import Node
import numpy as np
import csv
import pandas as pd

# SET PYTHONPATH
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.append('/home/dlee/ros2_ws/src/sar_simulation')

from sar_env import SAR_Sim_Interface

import matplotlib.pyplot as plt
import matplotlib as mpl
from scipy.interpolate import griddata

YELLOW = '\033[93m'
RED = '\033[91m'
GREEN = '\033[92m'
BLUE = "\033[34m"  
RESET = "\033[0m"  # Reset to default color

        
def main(args=None):

    ## DEFINE BASE PATH
    workspace_path = os.path.expanduser('~/ros2_ws')
    BASE_PATH = os.path.join(workspace_path, 'src', 'sar_simulation')
    LOG_DIR = f"{BASE_PATH}/sar_general" 

    fileName = "PolicyPerformance_Data_Tau_CR_0.255.csv"
    filePath = os.path.join(LOG_DIR,fileName)

    df = pd.read_csv(filePath, sep=',', comment="#")

    if 'V_mag' in df.columns and 'V_angle' in df.columns and 'R_legs' in df.columns:
        

        df2 = df.groupby(["V_mag", "V_angle"]).mean().round(3).reset_index()
    else:
        raise KeyError("Required columns ('V_mag', 'V_angle', 'reward_vals') are missing in the file")


    ## COLLECT DATA
    R = df2.iloc[:]['V_mag']
    Theta = df2.iloc[:]['V_angle']
    C = df2.iloc[:]['R_legs']

    ## DEFINE INTERPOLATION GRID
    R_list = np.linspace(R.min(),R.max(),num=50,endpoint=True).reshape(1,-1)
    Theta_list = np.linspace(Theta.min(),Theta.max(),num=50,endpoint=True).reshape(1,-1)
    R_grid, Theta_grid = np.meshgrid(R_list, Theta_list)

    ## INTERPOLATE DATA
    LR_interp = griddata((R, Theta), C, (R_list, Theta_list.T), method='linear')
    LR_interp += 0.001

    ## INIT PLOT INFO
    fig = plt.figure(figsize=(4,4))
    ax = fig.add_subplot(projection='polar')
    cmap = mpl.cm.jet
    norm = mpl.colors.Normalize(vmin=0,vmax=1)

    ax.contourf(np.radians(Theta_grid),R_grid,LR_interp,cmap=cmap,norm=norm,levels=60)

    ax.set_xticks(np.radians(np.arange(15,180,15)))
    ax.set_thetamin(max(15,15))
    ax.set_thetamax(min(90,90))

    ax.set_rticks([0.0,1.0,2.0,3.0,4.0,5.0])
    ax.set_rmin(0)
    ax.set_rmax(R.max())

    ## SAVE FIGURE WITH TEXT
    config_str = f"Trg_Tau_CR: 0.0.255 \
        \nTrg_Acc: 88.5" 
    fig.text(0,1,config_str,transform=plt.gcf().transFigure,ha='left',va='top',fontsize=6)


    plt.show(block=True)

if __name__ == '__main__':
    main()
