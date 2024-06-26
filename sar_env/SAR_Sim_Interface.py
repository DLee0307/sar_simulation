#!/usr/bin/env python3
import numpy as np
import sys
import os
import rclpy
from rclpy.node import Node

from threading import Thread,Event
import subprocess
import time

#SET PYTHONPATH 
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from sar_env import SAR_Base_Interface

YELLOW = '\033[93m'
RED = '\033[91m'
GREEN = '\033[92m'
CYAN = '\033[96m'
ENDC = '\033[m'

class SAR_Sim_Interface(SAR_Base_Interface):

    def __init__(self, GZ_Timeout=False):
        super().__init__()
        #self.loadSimParams()
        self.node = self
        self.SAR_Config = "default_config"

        self.GZ_Sim_process = None
        self.SAR_DC_process = None
        self.SAR_Ctrl_process = None

        self.GZ_ping_ok = False
        self.SAR_DC_ping_ok = False
        self.SAR_Ctrl_ping_ok = False
        self.NaN_check_ok = False
        self.Sim_Status = "Initializing"
        self.Sim_Restarting = False


        ## START SIMULATION
        self.Clock_Check_Flag = Event() # Stops clock monitoring during launch process
        self._restart_Sim()

    def loadSimParams(self):
        print()

    # =========================
    ##     ENV INTERACTION
    # =========================
    

    def _iterStep(self,n_steps:int = 10):
        print()

    def _getTick(self):
        print()

    def _getObs(self):        
        print()

    def sleep(self,time_s):        
        print()

    def Sim_VelTraj(self,pos,vel):         
        print()

    def resetPose(self,z_0=0.5): 
        print()

    def _setModelState(self,pos=[0,0,0.5],quat=[0,0,0,1],vel=[0,0,0],ang_vel=[0,0,0]):        
        print()

    def _setModelInertia(self,Mass=0,Inertia=[0,0,0]):        
        print()

    def scaleValue(self,x, original_range=(-1, 1), target_range=(-1, 1)):        
        print()

    # ============================
    ##      Command Handlers 
    # ============================
        
    ## ========== GAZEBO FUNCTIONS ==========
    def handle_Load_Params(self):

        print("Reset ROS Parameters\n")
        self.loadBaseParams()
        self.loadSimParams()
        self.sendCmd("Load_Params")
        
    def handle_GZ_StickyPads(self):
        cmd_flag = self.userInput("Turn sticky pads On/Off (1,0): ",int)
        self.sendCmd("GZ_StickyPads",cmd_flag=cmd_flag)

    def handle_GZ_Pose_Reset(self):
        print("Reset Pos/Vel -- Sticky off -- Controller Reset\n")
        self.resetPose()
        self.pausePhysics(pause_flag=False)

    def handle_GZ_Global_Vel_traj(self):

        ## GET GLOBAL VEL CONDITIONS 
        V_mag,V_angle = self.userInput("Flight Velocity (V_mag,V_angle):",float)

        ## CALC GLOBAL VELOCITIES
        Vx = V_mag*np.cos(np.radians(V_angle))
        Vy = 0
        Vz = V_mag*np.sin(np.radians(V_angle))
        V_B_O = [Vx,Vy,Vz]

        self.Sim_VelTraj(self.r_B_O,V_B_O)
        self.pausePhysics(False)

    def handle_GZ_Rel_Vel_traj(self):

        ## GET RELATIVE VEL CONDITIONS 
        V_mag,V_angle = self.userInput("Flight Velocity (V_mag,V_angle):",float)

        ## CALC RELATIVE VELOCITIES
        V_tx = V_mag*np.cos(np.radians(V_angle))
        V_ty = 0
        V_perp = V_mag*np.sin(np.radians(V_angle))

        ## CALCULATE GLOBAL VELOCITIES
        V_B_O = self.R_PW(np.array([V_tx,V_ty,V_perp]),self.Plane_Angle_rad)

        self.Sim_VelTraj(self.r_B_O,V_B_O)
        self.pausePhysics(False)

    # ================================
    ##     SIM MONITORING/LAUNCH
    # ================================
    def _launch_GZ_Sim(self):
        print()

    def _launch_SAR_DC(self):
        cmd = "gnome-terminal --disable-factory --geometry 85x46+1050+0 -- ros2 run sar_data_converter SAR_DataConverter"
        self.SAR_DC_process = subprocess.Popen(cmd, shell=True)

    def _launch_Controller(self):
        cmd = "gnome-terminal --disable-factory --geometry 85x46+1050+0 -- ros2 run sar_control SAR_Controller"
        self.SAR_Ctrl_process = subprocess.Popen(cmd, shell=True)


        
    def _start_monitoring_subprocesses(self):
        print()

    def _monitor_subprocesses(self):
        print()

    def _wait_for_sim_running(self,timeout=600):
        print()

    def _ping_service(self, service_name,timeout=5,silence_errors=False):
        print()

    def _kill_Sim(self):
        print()

    def _restart_Sim(self):
        self.Clock_Check_Flag.clear()
        '''
        try:
            # Terminate all processes containing the string 'gz sim'
            subprocess.run(['pkill', '-f', 'gz sim'], check=True)
        except subprocess.CalledProcessError as e:
            print(f"Failed to kill processes or nodes: {e}")

        try:
            # Terminate ROS2 nodes using pkill
            subprocess.run(['pkill', '-f', 'SAR_Controller_Node'], check=True)
            subprocess.run(['pkill', '-f', 'SAR_DataConverter_Node'], check=True)
            time.sleep(1.0)
        except subprocess.CalledProcessError as e:
            print(f"Failed to kill processes or nodes: {e}")
        '''
        ## KILL ALL POTENTIAL NODE/SUBPROCESSES
        try:
            # Terminate all processes containing the string 'gz sim'
            subprocess.run(['pkill', '-f', 'gz sim'], check=True)
        except subprocess.CalledProcessError as e:
            print(f"Failed to kill processes or nodes: {e}")
            
        try:
            # Terminate ROS2 nodes
            nodes_to_kill = [
                '/SAR_Controller_Node', 
                '/SAR_DataConverter_Node'
            ]

            for node in nodes_to_kill:
                subprocess.run(['ros2', 'node', 'kill', node], check=True)
                time.sleep(1.0)

        except subprocess.CalledProcessError as e:
            print(f"Failed to kill processes or nodes: {e}")

                ## LAUNCH CONTROLLER
        self._launch_Controller()
        self._wait_for_node(node_name="SAR_Controller_Node",timeout=5,interval=0.25)

        ## LAUNCH SAR_DC
        self._launch_SAR_DC()
        self._wait_for_node(node_name="SAR_DataConverter_Node",timeout=5,interval=0.25)

        self.Clock_Check_Flag.set()

    def _wait_for_node(self, node_name, timeout=None, interval=1.0):
        start_time = time.time()

        while True:
            # Use the ros2 node list command to check if the node is running
            result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True)
            if node_name in result.stdout:
                break

            # Timeout check
            if timeout is not None and time.time() - start_time > timeout:
                print(f"Timeout reached while waiting for {node_name} to launch.")
                return False

            # Print waiting message and wait
            print(f"Waiting for {node_name} to fully launch...")
            time.sleep(interval)

        print(f"{node_name} has fully launched.")
        return True

    def pausePhysics(self,pause_flag=True):
        print()





    def spin(self):
        rclpy.spin(self.node)
    
    def destroy(self):
        self.node.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    env = SAR_Sim_Interface()

    try:
        env.spin()
    except KeyboardInterrupt:
        pass
    finally:
        env.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    