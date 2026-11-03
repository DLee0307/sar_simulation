#!/usr/bin/env python3

import threading
import os
import sys
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor
import numpy as np

# SET PYTHONPATH
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.append('/home/dlee/ros2_ws/src/sar_simulation')

from sar_env import SAR_Sim_Interface

YELLOW = '\033[93m'
RED = '\033[91m'
GREEN = '\033[92m'
BLUE = '\033[34m'
RESET = '\033[0m'


def cmd_send(env, stop_event):
    command_handlers = {
        0: env.handle_Ctrl_Reset,
        1: env.handle_Pos_Cmd,
        2: env.handle_Vel_Cmd,
        3: env.handle_DH_Reset,
        5: env.handle_Stop,
        # 7: env.handle_Ang_Accel,
        8: env.handle_Policy,
        9: env.handle_Plane_Pose,
        10: env.handle_P2P_traj,
        11: env.handle_Global_Vel_traj,
        # 12: env.handle_Rel_Vel_traj,
        13: env.handle_Impact_traj,
        20: env.handle_Tumble_Detect,
        # 21: env.handle_Load_Params,
        22: env.handle_Start_Logging,
        # 23: env.handle_Cap_Logging,
        24: env.handle_Arm_Quad,
        # 30: env.handle_Thrust_CMD,
        # 31: env.handle_Motor_CMD,
        89: env.handle_GZ_Global_Vel_traj,
        # 90: env.handle_GZ_Pose_Reset,
        91: env.handle_GZ_StickyPads,
        92: env.handle_Optical_Flow_Flag,
        93: env.handle_Rolling_Shutter_Flag,
        # 94: env.handle_GZ_Rel_Vel_traj,
        95: env.handle_test_policy,
        96: env.handle_test_policy_,
        98: env.handle_OpticalFlow_Accuracy,
    }

    while not stop_event.is_set():
        try:
            print("\n========== Command Types ==========")
            print("0: Ctrl_Reset   8: Policy              20: Tumble_Detect      24: Arm_Quad")
            print("1: Pos          9: Plane_Pose          22: Start_Logging      89: GZ_Global_Vel_traj")
            print("2: Vel         10: P2P_traj            91: GZ_StickyPads      92: Optical_Flow_Flag")
            print("3: DH_Reset    11: Global_Vel_traj     93: Rolling_Shutter_Flag")
            print("5: Stop        13: Impact_traj         95: test_policy")
            print("                                            96: test_policy_")
            print("                                            98: OpticalFlow_Accuracy")

            cmd = env.userInput("\nCmd: ", int)

            if stop_event.is_set():
                break

            if cmd in command_handlers:
                command_handlers[cmd]()
            else:
                print(YELLOW + "Invalid Command: Try again" + RESET)

        except EOFError:
            print(YELLOW + "\nEOF received. Exiting command thread." + RESET)
            break

        except KeyboardInterrupt:
            print(YELLOW + "\nKeyboardInterrupt in command thread. Exiting..." + RESET)
            break

        except Exception as e:
            print(f"{RED}Error: {e}{RESET}")
            print(YELLOW + "INVALID INPUT: Try again" + RESET)
            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)

    # INIT GAZEBO ENVIRONMENT
    env = SAR_Sim_Interface(GZ_Timeout=False)
    env.initialize_sim()
    env.pausePhysics(False)
    # env.adjustSimSpeed(0.7)

    # INITIALIZE LOGGING DATA
    trial_num = 24
    env.Log_Name = f"Control_Playground--trial_{int(trial_num):02d}--{env.SAR_Config}.csv"

    stop_event = threading.Event()
    cmd_thread = threading.Thread(
        target=cmd_send,
        args=(env, stop_event),
        daemon=True
    )
    cmd_thread.start()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(env.node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        print(YELLOW + "\nKeyboardInterrupt received in main. Shutting down..." + RESET)

    finally:
        stop_event.set()

        try:
            executor.shutdown()
        except Exception as e:
            print(f"{RED}Executor shutdown error: {e}{RESET}")

        try:
            env.destroy()
        except Exception as e:
            print(f"{RED}Env destroy error: {e}{RESET}")

        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"{RED}rclpy shutdown error: {e}{RESET}")


if __name__ == '__main__':
    main()