# ## IMPORT ENVIRONMENTS
# from RL_Manager_DH import RL_Training_Manager
# from Envs.SAR_Sim_DeepRL_DH import SAR_Sim_DeepRL

# ## STANDARD IMPORTS
# import os
# from datetime import datetime
# import rospkg
# import argparse
# import json
# import rclpy

# # For run this code I need to command below in terminal.
# # cd ros2_ws/src/sar_simulation/sar_projects/NewDeepRL
# # python3 T3_Policy_Data_Collection_DH.py --TrainConfig /home/dlee/ros2_ws/src/sar_simulation/sar_projects/NewDeepRL/Config_Files/SOV5_3D_Sim/Data_SOV5_A30_L200_0deg_aRot90_S3D_DH.json
# # python3 T3_Policy_Data_Collection_DH.py --TrainConfig /home/dlee/ros2_ws/src/sar_simulation/sar_projects/NewDeepRL/Config_Files/SOV5OPENMV_3D_Sim/Data_SOV5OPENMV_A15_L250_0deg_aRot90_S3D_DH.json

# ## DEFINE BASE PATH
# workspace_path = os.path.expanduser('~/ros2_ws')
# BASE_PATH = os.path.join(workspace_path, 'src', 'sar_simulation')
# LOG_DIR = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs" 

# ## ARGUMENT PARSER
# parser = argparse.ArgumentParser(description='Policy Pre-Training Script')
# parser.add_argument('--GroupName',      help='Log group name', default='')
# parser.add_argument('--TrainConfig',    help='Path to training config file', required=True)
# parser.add_argument('--t_step_load',    help='Time step to load model', default=None, type=int)
# parser.add_argument('--S3_Upload',      help='Upload to S3', default=False, type=bool)
# args = parser.parse_args()


# ## LOAD TRAINING MODEL CONFIGURATION
# with open(args.TrainConfig, 'r') as TrainConfig_File:
#     TrainConfig = json.load(TrainConfig_File)

# ## UPDATE SAR TYPE AND SAR CONFIG IN BASE SETTINGS FILE
# Base_Settings_yaml = f"{BASE_PATH}/sar_config/Base_Settings.yaml"
# with open(Base_Settings_yaml, 'r') as file:
#     lines = file.readlines()

# for i, line in enumerate(lines):
#     if line.strip().startswith('SAR_Type:'):
#         lines[i] = f"  SAR_Type: '{TrainConfig['SAR_SETTINGS']['SAR_Type']}'\n"
#     elif line.strip().startswith('SAR_Config:'):
#         lines[i] = f"  SAR_Config: '{TrainConfig['SAR_SETTINGS']['SAR_Config']}'\n"

# with open(Base_Settings_yaml, 'w') as file:
#     file.writelines(lines)


# if __name__ == '__main__':

#     rclpy.init()

#     ## LOGGING CONFIGURATION
#     LogName = TrainConfig['LogName']
#     PlaneAngle = TrainConfig['ENV_KWARGS']['Plane_Angle_range'][0]
#     if args.t_step_load == None:
#         t_step_load = TrainConfig['t_step_optim']
#     else:
#         t_step_load = args.t_step_load

    
#     ## SELECT ENVIRONMENT
#     if TrainConfig['ENV_Type'] == "SAR_2D_Env":
#         env = SAR_Sim_DeepRL
#     elif TrainConfig['ENV_Type'] == "SAR_Sim_DeepRL":
#         env = SAR_Sim_DeepRL

#     ## SET UP TRAINING CONDITIONS FROM CONFIG
#     env_kwargs = TrainConfig['ENV_KWARGS']

#     ## CREATE RL MANAGER
#     RL_Manager = RL_Training_Manager(env,args.GroupName,TrainConfig['LogName'],env_kwargs=env_kwargs,S3_Upload=args.S3_Upload)
#     RL_Manager.env.Fine_Tuning_Flag = False
    
#     ## CREATE MODEL AND TRAIN
#     RL_Manager.create_model()
#     RL_Manager.load_model(
#         GroupName=args.GroupName,
#         LogName=TrainConfig['LogName'],
#         t_step_load=t_step_load,
#         Params_only=True,
#     )

#     RL_Manager.collect_landing_performance(
#         Plane_Angle_Step=45,
#         V_mag_Step=0.25,
#         V_angle_Step=5,
#         n=1,
#     )
    
#     # for i in [0,45,90,135,180]:
#     #     try:
#     #         RL_Manager.plot_landing_performance(PlaneAngle=i,saveFig=True,showFig=False)
#     #     except:
#     #         pass
#     # RL_Manager.save_NN_to_C_header()

#!/usr/bin/env python3

## IMPORT ENVIRONMENTS
from RL_Manager_DH import RL_Training_Manager
from Envs.SAR_Sim_DeepRL_DH import SAR_Sim_DeepRL

## STANDARD IMPORTS
import os
from datetime import datetime
import argparse
import json
import rclpy

## ERROR ALARM IMPORTS
import subprocess
import shutil
import traceback
import time


# For run this code I need to command below in terminal.
# cd ros2_ws/src/sar_simulation/sar_projects/NewDeepRL
# python3 T3_Policy_Data_Collection_DH.py --TrainConfig /home/dlee/ros2_ws/src/sar_simulation/sar_projects/NewDeepRL/Config_Files/SOV5_3D_Sim/Data_SOV5_A30_L200_0deg_aRot90_S3D_DH.json
# python3 T3_Policy_Data_Collection_DH.py --TrainConfig /home/dlee/ros2_ws/src/sar_simulation/sar_projects/NewDeepRL/Config_Files/SOV5OPENMV_3D_Sim/Data_SOV5OPENMV_A15_L250_0deg_aRot90_S3D_DH.json


## DEFINE BASE PATH
workspace_path = os.path.expanduser('~/ros2_ws')
BASE_PATH = os.path.join(workspace_path, 'src', 'sar_simulation')
LOG_DIR = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs"


## ARGUMENT PARSER
parser = argparse.ArgumentParser(description='Policy Data Collection Script')
parser.add_argument('--GroupName', help='Log group name', default='')
parser.add_argument('--TrainConfig', help='Path to training config file', required=True)
parser.add_argument('--t_step_load', help='Time step to load model', default=None, type=int)
parser.add_argument('--S3_Upload', help='Upload to S3', default=False, type=bool)
args = parser.parse_args()


def send_desktop_notification(title, message):
    """
    Send Ubuntu desktop notification if notify-send is available.
    """
    if shutil.which("notify-send") is not None:
        try:
            subprocess.Popen(
                ["notify-send", title, message],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
        except Exception:
            pass


def play_error_alarm(repeat=300):
    """
    Strong alarm for policy data collection crash.

    repeat=300 with sleep=3.0 means approximately 15 minutes of repeated alarm.
    Since spd-say works on your laptop, this uses spd-say as the main alarm.
    """

    # 1) Try to unmute and increase system volume
    if shutil.which("pactl") is not None:
        try:
            subprocess.run(
                ["pactl", "set-sink-mute", "@DEFAULT_SINK@", "0"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            subprocess.run(
                ["pactl", "set-sink-volume", "@DEFAULT_SINK@", "150%"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
        except Exception:
            pass

    elif shutil.which("amixer") is not None:
        try:
            subprocess.run(
                ["amixer", "set", "Master", "unmute"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            subprocess.run(
                ["amixer", "set", "Master", "100%"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
        except Exception:
            pass

    # 2) Repeat voice alarm
    alarm_text = "Wake up. Policy data collection crashed. Check the terminal."

    for i in range(repeat):
        print(
            f"[ALARM] Policy data collection crashed. Alarm {i + 1}/{repeat}",
            flush=True
        )

        if shutil.which("spd-say") is not None:
            try:
                subprocess.Popen(
                    ["spd-say", alarm_text],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
            except Exception:
                print("\a", end="", flush=True)
        else:
            # Fallback: terminal bell
            print("\a", end="", flush=True)

        time.sleep(3.0)


def write_error_log(error):
    """
    Save crash information to a local text file.
    """
    error_log_path = os.path.join(
        BASE_PATH,
        "sar_projects",
        "NewDeepRL",
        "policy_data_collection_crash_log.txt"
    )

    try:
        with open(error_log_path, "a") as f:
            f.write("\n" + "=" * 100 + "\n")
            f.write(f"Crash time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Error message: {str(error)}\n")
            f.write("-" * 100 + "\n")
            f.write(traceback.format_exc())
            f.write("\n")
    except Exception:
        pass


def load_train_config(train_config_path):
    """
    Load training/data collection configuration JSON.
    """
    with open(train_config_path, 'r') as TrainConfig_File:
        TrainConfig = json.load(TrainConfig_File)

    return TrainConfig


def update_base_settings_from_config(TrainConfig):
    """
    Update SAR_Type and SAR_Config in Base_Settings.yaml.
    """
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

    rclpy_initialized = False

    try:
        ## LOAD TRAINING MODEL CONFIGURATION
        TrainConfig = load_train_config(args.TrainConfig)

        ## UPDATE SAR TYPE AND SAR CONFIG IN BASE SETTINGS FILE
        update_base_settings_from_config(TrainConfig)

        ## INITIALIZE ROS2
        rclpy.init()
        rclpy_initialized = True

        ## LOGGING CONFIGURATION
        LogName = TrainConfig['LogName']
        PlaneAngle = TrainConfig['ENV_KWARGS']['Plane_Angle_range'][0]

        if args.t_step_load is None:
            t_step_load = TrainConfig['t_step_optim']
        else:
            t_step_load = args.t_step_load

        print("\n" + "=" * 100)
        print("Policy Data Collection Started")
        print("=" * 100)
        print(f"TrainConfig: {args.TrainConfig}")
        print(f"GroupName: {args.GroupName}")
        print(f"LogName: {LogName}")
        print(f"t_step_load: {t_step_load}")
        print(f"PlaneAngle: {PlaneAngle}")
        print(f"ENV_KWARGS: {TrainConfig['ENV_KWARGS']}")
        print("=" * 100 + "\n")

        ## SELECT ENVIRONMENT
        if TrainConfig['ENV_Type'] == "SAR_2D_Env":
            env = SAR_Sim_DeepRL

        elif TrainConfig['ENV_Type'] == "SAR_Sim_DeepRL":
            env = SAR_Sim_DeepRL

        else:
            raise ValueError(f"Unknown ENV_Type: {TrainConfig['ENV_Type']}")

        ## SET UP TRAINING CONDITIONS FROM CONFIG
        env_kwargs = TrainConfig['ENV_KWARGS']

        ## CREATE RL MANAGER
        RL_Manager = RL_Training_Manager(
            env,
            args.GroupName,
            TrainConfig['LogName'],
            env_kwargs=env_kwargs,
            S3_Upload=args.S3_Upload
        )

        ## Make sure performance collection uses sweep conditions,
        ## not fine-tuning CSV conditions.
        RL_Manager.env.Fine_Tuning_Flag = False

        ## CREATE MODEL STRUCTURE
        RL_Manager.create_model()

        ## LOAD TRAINED MODEL PARAMETERS
        RL_Manager.load_model(
            GroupName=args.GroupName,
            LogName=TrainConfig['LogName'],
            t_step_load=t_step_load,
            Params_only=True,
        )

        ## COLLECT LANDING PERFORMANCE DATA
        RL_Manager.collect_landing_performance(
            Plane_Angle_Step=45,
            V_mag_Step=0.25,
            V_angle_Step=5,
            n=2,
        )

        print("\n" + "=" * 100)
        print("Policy data collection completed successfully.")
        print("=" * 100 + "\n")

        # Optional plotting / export
        # for i in [0,45,90,135,180]:
        #     try:
        #         RL_Manager.plot_landing_performance(
        #             PlaneAngle=i,
        #             saveFig=True,
        #             showFig=False
        #         )
        #     except Exception:
        #         pass
        #
        # RL_Manager.save_NN_to_C_header()

    except KeyboardInterrupt:
        print("\nPolicy data collection stopped by user. No alarm triggered.")

    except Exception as e:
        print("\n" + "=" * 100)
        print("[ERROR] Policy data collection crashed.")
        print(f"Error message: {e}")
        print("=" * 100)

        traceback.print_exc()
        write_error_log(e)

        send_desktop_notification(
            "Policy Data Collection Crashed",
            "Policy data collection crashed. Check the terminal."
        )

        try:
            play_error_alarm(repeat=300)
        except KeyboardInterrupt:
            print("\nAlarm stopped by user.")

        # Re-raise so the terminal still shows that the script failed
        raise

    finally:
        try:
            if rclpy_initialized and rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass