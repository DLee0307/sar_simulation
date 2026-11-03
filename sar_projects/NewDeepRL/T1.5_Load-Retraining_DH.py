# ## IMPORT ENVIRONMENTS
# from RL_Manager_DH import RL_Training_Manager
# from Envs.SAR_Sim_DeepRL_DH import SAR_Sim_DeepRL

# ## STANDARD IMPORTS
# import os
# from datetime import datetime
# import argparse
# import json
# import rclpy

# # For run this code I need to command below in terminal.
# # cd ros2_ws/src/sar_simulation/sar_projects/NewDeepRL
# # python3 T1.5_Load-Retraining_DH.py 
# # Now A15_L250


# ## DEFINE BASE PATH
# workspace_path = os.path.expanduser('~/ros2_ws')
# BASE_PATH = os.path.join(workspace_path, 'src', 'sar_simulation')

# if __name__ == '__main__':
#     rclpy.init()

#     # 환경 설정
#     env_kwargs = {
#         "Ang_Acc_range": [-90.0, -89.9],
#         "Plane_Angle_range": [0, 0],
#         "V_mag_range": [1.0, 5.0],
#         "V_angle_range": [90.0, 90.0],
#         "Render": False,
#         "GZ_Timeout": True
#     }
#     RL_Manager = RL_Training_Manager(
#         env=SAR_Sim_DeepRL, 
#         Group_Name='', 
#         Log_Name='SOV5OPENMV_A15_L250_0deg_aRot90_S3D', 
#         env_kwargs=env_kwargs
#     )

#     # 저장된 모델 및 리플레이 버퍼 로드0
#     RL_Manager.load_model(
#         t_step_load=27000, 
#         GroupName='', 
#         LogName='SOV5OPENMV_A15_L250_0deg_aRot90_S3D', 
#         Params_only=False, 
#         load_replay_buffer=True
#     )

#     # 학습 재개
#     RL_Manager.train_model(
#         reset_timesteps=False, 
#         t_step_max=96000
#     )

#     rclpy.shutdown()

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
# python3 T1.5_Load-Retraining_DH.py
# Now A15_L250


## DEFINE BASE PATH
workspace_path = os.path.expanduser('~/ros2_ws')
BASE_PATH = os.path.join(workspace_path, 'src', 'sar_simulation')


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
    Strong alarm for training crash.

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
    alarm_text = "Wake up. RL training crashed. Check the terminal."

    for i in range(repeat):
        print(f"[ALARM] Training crashed. Alarm {i + 1}/{repeat}", flush=True)

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
        "training_crash_log.txt"
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


if __name__ == '__main__':

    rclpy_initialized = False

    try:
        rclpy.init()
        rclpy_initialized = True

        # 환경 설정
        env_kwargs = {
            "Ang_Acc_range": [-90.0, -89.9],
            "Plane_Angle_range": [0, 0],
            "V_mag_range": [0.75, 2.0],
            "V_angle_range": [15.0, 90.0],
            "Render": False,
            "GZ_Timeout": True,
            "Obs_Mode": "TauTheta"
        }

        RL_Manager = RL_Training_Manager(
            env=SAR_Sim_DeepRL,
            Group_Name='',
            Log_Name='SOV5OPENMV_A15_L250_0deg_aRot90_S3D',
            env_kwargs=env_kwargs
        )

        # 저장된 모델 및 리플레이 버퍼 로드
        RL_Manager.load_model(
            t_step_load=21400,
            GroupName='',
            LogName='SOV5OPENMV_A15_L250_0deg_aRot90_S3D',
            Params_only=False,
            load_replay_buffer=True
        )

        # 학습 재개
        RL_Manager.train_model(
            reset_timesteps=False,
            t_step_max=96000
        )

        print("\nTraining completed successfully.")

    except KeyboardInterrupt:
        print("\nTraining stopped by user. No alarm triggered.")

    except Exception as e:
        print("\n" + "=" * 100)
        print("[ERROR] RL training crashed.")
        print(f"Error message: {e}")
        print("=" * 100)

        traceback.print_exc()
        write_error_log(e)

        send_desktop_notification(
            "RL Training Crashed",
            "RL training crashed. Check the terminal."
        )

        # Strong alarm: about 15 minutes
        play_error_alarm(repeat=300)

        # Re-raise so the terminal still shows that the script failed
        raise

    finally:
        try:
            if rclpy_initialized and rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass