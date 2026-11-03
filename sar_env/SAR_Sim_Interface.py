#!/usr/bin/env python3
import numpy as np

import os
import time
import subprocess
from threading import Thread, Event, Lock
import rclpy
from rclpy.node import Node
from rclpy.task import Future
import sys

from sar_msgs.srv import CTRLCmdSrv
from sar_msgs.srv import CTRLGetObs
from sar_msgs.srv import CTRLGetTcon

import csv
import threading

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

        ##!! Need to analyze
        self.node = self # If remove this cannot send command?
        self.Clock_Check_Flag = Event() # Stops clock monitoring during launch process
        #self.SAR_Config = "default_config"

        self.GZ_Sim_process = None
        self.SAR_DC_process = None
        self.SAR_Ctrl_process = None
        self.pause_simulation_process = None # For pausing simulation
        self.adjust_simulation_speed_process = None

        self.GZ_ping_ok = False
        self.SAR_DC_ping_ok = False
        self.SAR_Ctrl_ping_ok = False
        self.NaN_check_ok = False
        self.Sim_Status = "Initializing"
        self.Sim_Restarting = False

        self.sim_restart_lock = Lock()
        self.is_restarting_sim = False
        self.monitor_thread = None


    def initialize_sim(self):
        self.Sim_Status = "Starting"

        ok = self._restart_sim_safely(ready_timeout=25.0)
        if not ok:
            raise RuntimeError("Simulation did not become fully ready in time.")

        self.Sim_Status = "Running"


    def loadSimParams(self):
        print()

    # =========================
    ##     ENV INTERACTION
    # =========================
    

    def _iterStep(self, n_steps: int = 10):
        self.pausePhysics(True)
        time.sleep(0.25)

        cmd = [
            'gz', 'service', '-s', '/world/empty/control',
            '--reqtype', 'gz.msgs.WorldControl',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000',
            '--req', f'pause: true, multi_step: {n_steps}'
        ]
        result = subprocess.run(cmd, capture_output=True, text=True)
        return result.returncode == 0


    def _getTick(self):
        srv = CTRLGetObs.Request()
        result = self.callService('/CTRL/Get_Obs', srv, CTRLGetObs)
        if result is None:
            raise RuntimeError("_getTick failed: /CTRL/Get_Obs returned None")
        
        time.sleep(0.25)

        return result.tick

    def _getObs_Tau(self):        
        resp = self.callService('/CTRL/Get_Obs',CTRLGetObs.Request(),CTRLGetObs)
        
        Tau_DH = resp.tau_dh
        Theta_x_DH = resp.theta_x_dh

        Tau_CR = resp.tau_cr
        Theta_x = resp.theta_x
        D_perp_CR = resp.d_perp_cr

        Tau_CM = resp.tau_cm

        if np.isnan(Tau_DH):
            Tau_DH = 5

        else:
            Tau_DH = np.clip(Tau_DH, -5, 5)

        if np.isnan(Theta_x_DH):
            Theta_x_DH = 20

        else:
            Theta_x_DH = np.clip(Theta_x_DH, -20, 20)

        if np.isnan(Tau_CR):
            Tau_CR = 5

        else:
            Tau_CR = np.clip(Tau_CR, -5, 5)

        if np.isnan(Theta_x):
            Theta_x = 20

        else:
            Theta_x = np.clip(Theta_x, -20, 20)

        if np.isnan(Tau_CM):
            Tau_CM = 5

        else:
            Tau_CM = np.clip(Tau_CM, -5, 5)

        if np.isnan(D_perp_CR):
            D_perp_CR = 2.0

        else:
            D_perp_CR = np.clip(D_perp_CR, -0.5, 2.0)


        return Tau_DH, Theta_x_DH, Tau_CR, Theta_x, D_perp_CR, Tau_CM


    def _getObs(self):        
        resp = self.callService('/CTRL/Get_Obs',CTRLGetObs.Request(),CTRLGetObs)

        Tau_CR = resp.tau_cr
        Theta_x = resp.theta_x
        D_perp_CR = resp.d_perp_cr
        Plane_Angle_rad = np.radians(resp.plane_angle_deg)

        obs_list = [Tau_CR,Theta_x,D_perp_CR,self.Plane_Angle_rad]

        Tau_CR_scaled = self.scaleValue(Tau_CR,original_range=[-5,5],target_range=[-1,1])
        Theta_x_scaled = self.scaleValue(Theta_x,original_range=[-20,20],target_range=[-1,1])
        D_perp_CR_scaled = self.scaleValue(D_perp_CR,original_range=[-0.5,2.0],target_range=[-1,1])
        Plane_Angle_scaled = self.scaleValue(self.Plane_Angle_deg,original_range=[0,180],target_range=[-1,1])

        scaled_obs_list = [Tau_CR_scaled,Theta_x_scaled,D_perp_CR_scaled,Plane_Angle_scaled]

        ## OBSERVATION VECTOR
        obs = np.array(scaled_obs_list,dtype=np.float32)

        #print("obs : ", obs)

        return obs

    def _safe_clip(self, value, default_value, min_value, max_value):
        """
        Use default_value if the input is NaN or invalid.
        Otherwise, clip the value to the range [min_value, max_value].
        """
        try:
            if value is None or np.isnan(value):
                return default_value
            return float(np.clip(value, min_value, max_value))
        except Exception:
            return default_value


    # def _select_tau_theta_source(self, resp):
    #     """
    #     Select the Tau / Theta_x source based on
    #     CAM_SETTINGS.Optical_Flow_Source.

    #     Source mapping:
    #         Motion_Capture_System_Ori -> Tau_CM / Theta_x_CM
    #         Motion_Capture_System_Cam -> Tau_Cam_GT / Theta_x_Cam_GT
    #         Motion_Capture_System_CR  -> Tau_CR / Theta_x
    #         Camera                    -> Tau_DH / Theta_x_DH
    #     """

    #     source = getattr(self, "Optical_Flow_Source", "Motion_Capture_System_Ori")

    #     if source == "Camera":
    #         Tau = resp.tau_dh
    #         Theta_x = resp.theta_x_dh
    #         tau_label = "Tau_DH"
    #         theta_label = "Theta_x_DH"

    #     elif source == "Motion_Capture_System_CR":
    #         Tau = resp.tau_cr
    #         Theta_x = resp.theta_x
    #         tau_label = "Tau_CR"
    #         theta_label = "Theta_x"

    #     elif source == "Motion_Capture_System_Ori":
    #         Tau = resp.tau_cm
    #         Theta_x = resp.theta_x_cm
    #         tau_label = "Tau_CM"
    #         theta_label = "Theta_x_CM"

    #     elif source == "Motion_Capture_System_Cam":
    #         Tau = resp.tau_cam_gt
    #         Theta_x = resp.theta_x_cam_gt
    #         tau_label = "Tau_Cam_GT"
    #         theta_label = "Theta_x_Cam_GT"

    #         # Treat non-positive or near-zero camera-frame TTC as invalid/far.
    #         if Tau is None or np.isnan(Tau) or Tau <= 1e-3:
    #             Tau = 5.0

    #     else:
    #         valid_sources = [
    #             "Camera",
    #             "Motion_Capture_System_CR",
    #             "Motion_Capture_System_Ori",
    #             "Motion_Capture_System_Cam",
    #         ]

    #         error_msg = (
    #             f"Invalid Optical_Flow_Source='{source}'. "
    #             f"Please check CAM_SETTINGS.Optical_Flow_Source in the YAML file. "
    #             f"Valid options are: {valid_sources}"
    #         )

    #         self.get_logger().error(error_msg)
    #         raise ValueError(error_msg)

    #     Tau = self._safe_clip(
    #         Tau,
    #         default_value=5.0,
    #         min_value=-5.0,
    #         max_value=5.0
    #     )

    #     Theta_x = self._safe_clip(
    #         Theta_x,
    #         default_value=20.0,
    #         min_value=-20.0,
    #         max_value=20.0
    #     )

    #     Tau_scaled = self.scaleValue(
    #         Tau,
    #         original_range=[-5, 5],
    #         target_range=[-1, 1]
    #     )

    #     Theta_x_scaled = self.scaleValue(
    #         Theta_x,
    #         original_range=[-20, 20],
    #         target_range=[-1, 1]
    #     )

    #     print(f"[Optical_Flow_Source: {source}] {tau_label}: {Tau:.3f}, scaled: {Tau_scaled:.3f}")
    #     print(f"[Optical_Flow_Source: {source}] {theta_label}: {Theta_x:.3f}, scaled: {Theta_x_scaled:.3f}")

    #     return np.array([Tau_scaled, Theta_x_scaled], dtype=np.float32)

    def _select_tau_theta_source(self, resp):
        """
        Select Tau / Theta_x source using CAM_SETTINGS.Optical_Flow_Source.

        Obs_Mode:
            TauTheta          -> [Tau, Theta_x]
            TauThetaDperpCam  -> [Tau, Theta_x, D_perp_Cam]
        """

        source = getattr(self, "Optical_Flow_Source", "Motion_Capture_System_Cam")

        if source == "Camera":
            Tau = resp.tau_dh
            Theta_x = resp.theta_x_dh
            tau_label = "Tau_DH"
            theta_label = "Theta_x_DH"

        elif source == "Motion_Capture_System_CR":
            Tau = resp.tau_cr
            Theta_x = resp.theta_x
            tau_label = "Tau_CR"
            theta_label = "Theta_x"

        elif source == "Motion_Capture_System_Ori":
            Tau = resp.tau_cm
            Theta_x = resp.theta_x_cm
            tau_label = "Tau_CM"
            theta_label = "Theta_x_CM"

        elif source == "Motion_Capture_System_Cam":
            Tau = resp.tau_cam_gt
            Theta_x = resp.theta_x_cam_gt
            tau_label = "Tau_Cam_GT"
            theta_label = "Theta_x_Cam_GT"

            # if Tau is None or np.isnan(Tau) or Tau <= 1e-3:
            #     Tau = 5.0

        else:
            valid_sources = [
                "Camera",
                "Motion_Capture_System_CR",
                "Motion_Capture_System_Ori",
                "Motion_Capture_System_Cam",
            ]
            raise ValueError(
                f"Invalid Optical_Flow_Source='{source}'. "
                f"Valid options are: {valid_sources}"
            )

        D_perp_Cam = resp.d_perp_cam

        Tau = self._safe_clip(
            Tau,
            default_value=5.0,
            min_value=-5.0,
            max_value=5.0
        )

        Theta_x = self._safe_clip(
            Theta_x,
            default_value=20.0,
            min_value=-20.0,
            max_value=20.0
        )

        D_perp_Cam = self._safe_clip(
            D_perp_Cam,
            default_value=2.0,
            min_value=-0.5,
            max_value=2.0
        )

        Tau_scaled = self.scaleValue(
            Tau,
            original_range=[-5, 5],
            target_range=[-1, 1]
        )

        Theta_x_scaled = self.scaleValue(
            Theta_x,
            original_range=[-20, 20],
            target_range=[-1, 1]
        )

        D_perp_Cam_scaled = self.scaleValue(
            D_perp_Cam,
            original_range=[-0.5, 2.0],
            target_range=[-1, 1]
        )

        print(f"[Obs_Mode: {self.Obs_Mode}]")
        print(f"[Optical_Flow_Source: {source}] {tau_label}: {Tau:.3f}, scaled: {Tau_scaled:.3f}")
        print(f"[Optical_Flow_Source: {source}] {theta_label}: {Theta_x:.3f}, scaled: {Theta_x_scaled:.3f}")

        if self.Obs_Mode == "TauTheta":
            return np.array(
                [Tau_scaled, Theta_x_scaled],
                dtype=np.float32
            )

        elif self.Obs_Mode == "TauThetaDperpCam":
            print(f"[Optical_Flow_Source: {source}] D_perp_Cam: {D_perp_Cam:.3f}, scaled: {D_perp_Cam_scaled:.3f}")

            return np.array(
                [Tau_scaled, Theta_x_scaled, D_perp_Cam_scaled],
                dtype=np.float32
            )

        else:
            raise ValueError(
                f"Invalid Obs_Mode='{self.Obs_Mode}'. "
                "Use 'TauTheta' or 'TauThetaDperpCam'."
            )

    def _getObs_DH(self):
        resp = self.callService(
            '/CTRL/Get_Obs',
            CTRLGetObs.Request(),
            CTRLGetObs
        )

        if resp is None:
            self.get_logger().warn("_getObs_DH failed: /CTRL/Get_Obs returned None")
            return np.ones(self.observation_space.shape, dtype=np.float32)

        obs = self._select_tau_theta_source(resp)

        if obs.shape != self.observation_space.shape:
            raise RuntimeError(
                f"Observation shape mismatch: obs.shape={obs.shape}, "
                f"observation_space.shape={self.observation_space.shape}. "
                f"Obs_Mode={self.Obs_Mode}"
            )

        return obs

    # def _getObs_DH(self):
    #     resp = self.callService(
    #         '/CTRL/Get_Obs',
    #         CTRLGetObs.Request(),
    #         CTRLGetObs
    #     )

    #     if resp is None:
    #         self.get_logger().warn("_getObs_DH failed: /CTRL/Get_Obs returned None")
    #         return np.array([1.0, 1.0], dtype=np.float32)

    #     obs = self._select_tau_theta_source(resp)

    #     return obs

#     def _getObs_DH(self):        
#         resp = self.callService('/CTRL/Get_Obs',CTRLGetObs.Request(),CTRLGetObs)

# #/////

#         # Tau_DH = resp.tau_dh
#         # Theta_x_DH = resp.theta_x_dh
#         # # Theta_y_DH = resp.theta_y_dh

#         # # Tau = resp.tau
#         # # print("Tau", Tau)

#         # # print("Tau_DH", Tau_DH)
#         # # print("Theta_x_DH", Theta_x_DH)
#         # # # print("Theta_y_DH", Theta_y_DH)
        
#         # if np.isnan(Tau_DH):
#         #     Tau_DH = 5

#         # else:
#         #     Tau_DH = np.clip(Tau_DH, -5, 5)

#         # if np.isnan(Theta_x_DH):
#         #     Theta_x_DH = 20

#         # else:
#         #     Theta_x_DH = np.clip(Theta_x_DH, -20, 20)

#         # # if np.isnan(Theta_y_DH):
#         # #     Theta_y_DH = 20

#         # # else:
#         # #     Theta_y_DH = np.clip(Theta_y_DH, -20, 20)

#         # Tau_DH_scaled = self.scaleValue(Tau_DH,original_range=[-5,5],target_range=[-1,1])
#         # Theta_x_DH_scaled = self.scaleValue(Theta_x_DH,original_range=[-20,20],target_range=[-1,1])
#         # # Theta_y_DH_scaled = self.scaleValue(Theta_y_DH,original_range=[-20,20],target_range=[-1,1])

#         # scaled_obs_list = [Tau_DH_scaled, Theta_x_DH_scaled]

# #/////

# #@@@@@
        
#         # Tau_CR = resp.tau_cr
#         # Theta_x = resp.theta_x
#         # D_perp_CR = resp.d_perp_cr

#         # # print("Tau_CR", Tau_CR)
#         # # print("Theta_x", Theta_x)        


#         # if np.isnan(Tau_CR):
#         #     Tau_CR = 5

#         # else:
#         #     Tau_CR = np.clip(Tau_CR, -5, 5)

#         # if np.isnan(Theta_x):
#         #     Theta_x = 20

#         # else:
#         #     Theta_x = np.clip(Theta_x, -20, 20)

#         # if np.isnan(D_perp_CR):
#         #     D_perp_CR = 2.0

#         # else:
#         #     D_perp_CR = np.clip(D_perp_CR, -0.5, 2.0)

#         # Tau_CR_scaled = self.scaleValue(Tau_CR,original_range=[-5,5],target_range=[-1,1])
#         # Theta_x_scaled = self.scaleValue(Theta_x,original_range=[-20,20],target_range=[-1,1])
#         # D_perp_CR_scaled = self.scaleValue(D_perp_CR,original_range=[-0.5,2.0],target_range=[-1,1])

#         # # scaled_obs_list = [Tau_CR_scaled, Theta_x_scaled]
#         # scaled_obs_list = [Tau_CR_scaled, Theta_x_scaled, D_perp_CR_scaled]

# #@@@@@@  
        
# #######

#         Tau_CM = resp.tau_cm
#         Theta_x_CM = resp.theta_x_cm

#         # print("Tau_CM", Tau_CM)
#         # print("Theta_x_CM", Theta_x_CM)   

#         if np.isnan(Tau_CM):
#             Tau_CM = 5

#         else:
#             Tau_CM = np.clip(Tau_CM, -5, 5)

#         if np.isnan(Theta_x_CM):
#             Theta_x_CM = 20

#         else:
#             Theta_x_CM = np.clip(Theta_x_CM, -20, 20)

#         Tau_CM_scaled = self.scaleValue(Tau_CM,original_range=[-5,5],target_range=[-1,1])
#         Theta_x_CM_scaled = self.scaleValue(Theta_x_CM,original_range=[-20,20],target_range=[-1,1])
        
#         scaled_obs_list = [Tau_CM_scaled, Theta_x_CM_scaled]

# #######


#         ## OBSERVATION VECTOR
#         obs = np.array(scaled_obs_list,dtype=np.float32)
# #/////
#         # print("Tau_DH_Scaled", obs[0])
#         # print("Theta_x_DH_Scaled", obs[1])
# #/////

# #@@@@@
#         # print("Tau_CR_Scaled", obs[0])
#         # print("Theta_x_Scaled", obs[1]) 
#         # print("D_perp_CR_Scaled", obs[2]) 
# #@@@@@


# ######
#         print("Tau_CM_Scaled", obs[0])
#         print("Theta_x_CM_Scaled", obs[1])
# ######

#         return obs

    def _get_Tcondition_Obs(self): # For getting Terminate and Truncate condition.
        resp = self.callService('/CTRL/Get_Tcon',CTRLGetTcon.Request(),CTRLGetTcon)
        
        self.r_B_O = np.round([resp.pose_b_o.position.x,
                                resp.pose_b_o.position.y,
                                resp.pose_b_o.position.z],3)
        return self.r_B_O


    def sleep(self,time_s):        
        """
        Sleep in terms of Gazebo sim seconds not real time seconds
        """

        t_start = self.t
        while self.t - t_start <= time_s:

            ## NEGATIVE TIME DELTA
            if self.t < t_start:
                self.Done = True
                return False

        return True

# Need to Change

    def Sim_VelTraj(self, pos, vel, t_total: float = 1.0):
        """
        Args:
            pos (list): Launch position [m]   | [x,y,z]
            vel (list): Launch velocity [m/s] | [Vx,Vy,Vz]
        """

        #print("Sim_VelTraj is started")

        pos = list(pos)
        vel = list(vel)

        model_name = self.SAR_Config
        x, y, z = pos[0], pos[1], pos[2]

        self.pausePhysics(pause_flag=True)

        ok = self.set_pose_gz([x, y, z], name=model_name)
        if not ok:
            self.get_logger().error("Sim_VelTraj failed: first set_pose_gz failed")
            return False

        ok = self.sendCmd('Pos', cmd_vals=pos, cmd_flag=1.0)
        if not ok:
            self.get_logger().error("Sim_VelTraj failed: Pos command failed")
            return False

        ok = self.step_and_wait(100)
        if not ok:
            self.get_logger().error("Sim_VelTraj failed: stabilization after Pos failed")
            return False

        self.pausePhysics(pause_flag=True)

        ok = self.set_pose_gz([x, y, z], name=model_name)
        if not ok:
            self.get_logger().error("Sim_VelTraj failed: second set_pose_gz failed")
            return False

        self.pausePhysics(pause_flag=True)

        ok = self.step_and_wait(1000)
        if not ok:
            self.get_logger().error("Sim_VelTraj failed: long stabilization failed")
            return False

        ok = self.sendCmd(
            'Const_Vel_traj',
            cmd_vals=[float(vel[0]), float(self.TrajAcc_Max[0]), float(self.TrajJerk_Max[0])],
            cmd_flag=0.0
        )
        if not ok:
            self.get_logger().error("Sim_VelTraj failed: x velocity command failed")
            return False

        ok = self.step_and_wait(2, tick_timeout=2.0, state_timeout=2.0)
        if not ok:
            self.get_logger().error("Sim_VelTraj failed: after x velocity command")
            return False

        ok = self.sendCmd(
            'Const_Vel_traj',
            cmd_vals=[float(vel[2]), float(self.TrajAcc_Max[2]), float(self.TrajJerk_Max[2])],
            cmd_flag=2.0
        )
        if not ok:
            self.get_logger().error("Sim_VelTraj failed: z velocity command failed")
            return False

        ok = self.step_and_wait(2, tick_timeout=2.0, state_timeout=2.0)
        if not ok:
            self.get_logger().error("Sim_VelTraj failed: after z velocity command")
            return False

        ok = self.sendCmd('Activate_traj', cmd_vals=[1.0, 1.0, 1.0])
        if not ok:
            self.get_logger().error("Sim_VelTraj failed: Activate_traj failed")
            return False

        ok = self.step_and_wait(6, tick_timeout=2.0, state_timeout=2.0)
        if not ok:
            self.get_logger().error("Sim_VelTraj failed: post activation sync failed")
            return False

        ok = self.step_and_wait(round(t_total * 1000), tick_timeout=15.0, state_timeout=5.0)
        if not ok:
            self.get_logger().error("Sim_VelTraj failed: main trajectory stepping failed")
            return False

        #print("Sim_VelTraj is done")

        return True

    # def Sim_VelTraj(self,pos,vel,t_total:float = 1.0):
    #     """
    #     Args:
    #         pos (list): Launch position [m]   | [x,y,z]
    #         vel (list): Launch velocity [m/s] | [Vx,Vy,Vz]
    #     """

    #     model_name = self.SAR_Config
    #     x, y, z = pos[0], pos[1], pos[2]

    #     self.pausePhysics(pause_flag=True)

    #     # Use subprocess to call the GZ service for setting pose
    #     ok = self.set_pose_gz([x, y, z], name=model_name)
    #     if not ok:
    #         self.get_logger().error("Sim_VelTraj failed: set_pose_gz failed")
    #         return False

    #     # Run the command
    #     self.sendCmd('Pos', cmd_vals=list(pos), cmd_flag=1.0)
    #     self._iterStep(100) # For stabilization


    #     self.pausePhysics(pause_flag=True)

    #     # Use subprocess to call the GZ service for setting pose - twice setting pose for stabilization
    #     ok = self.set_pose_gz([x, y, z], name=model_name)
    #     if not ok:
    #         self.get_logger().error("Sim_VelTraj failed: second set_pose_gz failed")
    #         return False

    #     ## PUBLISH MODEL STATE SERVICE REQUEST
    #     self.pausePhysics(pause_flag=True)
    #     #self.callService('/gazebo/set_model_state',state_srv,SetModelState)
    #     self._iterStep(1000) # for stabilization
    #     time.sleep(1)
        
    #     # ## SET DESIRED VEL IN CONTROLLER
    #     # making desired velocity via acceleration
    #     self.sendCmd('Const_Vel_traj',cmd_vals=[vel[0],self.TrajAcc_Max[0],self.TrajJerk_Max[0]],cmd_flag=0.0)
    #     self._iterStep(2)
    #     self.sendCmd('Const_Vel_traj',cmd_vals=[vel[2],self.TrajAcc_Max[2],self.TrajJerk_Max[2]],cmd_flag=2.0)
    #     self._iterStep(2)
    #     self.sendCmd('Activate_traj',cmd_vals=[1.0,1.0,1.0])
    #     self._iterStep(6) ## ROUND OUT TO 10 ITER STEPS (0.01s) TO MATCH 100Hz CONTROLLER 
    #     self._iterStep(round(t_total * 1000))

    def calOF_activation(self):

        #self.pausePhysics(pause_flag=True)
        rclpy.spin_once(self)
        self.adjustSimSpeed(0.1)
        self.sendCmd("Optical_Flow_Flag",cmd_vals=[1.0,1.0,1.0],cmd_flag=1.0)
        #self.pausePhysics(pause_flag=False)

    def calOF_unactivation(self):

        print()

    def set_pose_gz(self, pos, quat=(0.0, 0.0, 0.0, 1.0), name=None, timeout_ms=3000):
        if name is None:
            name = self.SAR_Config

        cmd = [
            'gz', 'service', '-s', '/world/empty/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', str(timeout_ms),
            '--req',
            (
                f'name: "{name}", '
                f'position: {{x: {float(pos[0])}, y: {float(pos[1])}, z: {float(pos[2])}}}, '
                f'orientation: {{x: {float(quat[0])}, y: {float(quat[1])}, z: {float(quat[2])}, w: {float(quat[3])}}}'
            )
        ]

        result = subprocess.run(cmd, capture_output=True, text=True)

        stdout = result.stdout.strip()
        stderr = result.stderr.strip()

        if result.returncode != 0:
            self.get_logger().error(
                f"set_pose_gz failed for model '{name}': stderr={stderr}"
            )
            return False

        # self.get_logger().info(
        #     f"set_pose_gz response for model '{name}': stdout={stdout}"
        # )

        return True


    # https://github.com/gazebosim/gz-sim/pull/2440
    # https://github.com/gazebosim/gz-sim/issues/2318
    def resetPose(self,z_0=0.4): 
        print("resetPose is started")
        rclpy.spin_once(self, timeout_sec=0.05)

        # reset 시작 직후 sim/controller warm-up
        ok = self.step_and_wait(20, tick_timeout=3.0, state_timeout=2.0)
        if not ok:
            self.get_logger().error("resetPose failed: initial warm-up step failed")
            return False

#/////

        # If the previous step was terminated while using the real camera optical flow,
        # deactivate the optical flow plugin and restore the simulation speed.
        
        # self.sendCmd("Optical_Flow_Flag",cmd_vals=[1.0,1.0,1.0],cmd_flag=0.0)
        # self.adjustSimSpeed(1.0)

        if getattr(self, "Optical_Flow_Source", "").strip() == "Camera":
            self.sendCmd(
                "Optical_Flow_Flag",
                cmd_vals=[1.0, 1.0, 1.0],
                cmd_flag=0.0
            )
            self.adjustSimSpeed(1.0)
 
#/////

        self.sendCmd('GZ_StickyPads',cmd_vals=[1.0,1.0,1.0],cmd_flag=0.0)
        self.sendCmd('Tumble_Detect',cmd_vals=[1.0,1.0,1.0],cmd_flag=0.0)
        self.sendCmd("Ctrl_Reset", cmd_vals=[1.0,1.0,1.0])
        self.sendCmd("DH_Reset", cmd_vals=[1.0,1.0,1.0])


        ok = self.set_pose_gz([0.0, 0.0, z_0])
        if not ok:
            self.get_logger().error("resetPose failed: temporary pose move failed")
            return False
        
        ok = self.step_and_wait(1000, tick_timeout=15.0, state_timeout=5.0)
        if not ok:
            self.get_logger().error("resetPose failed: first stabilization failed")
            return False

        #Detach the pad with ceiling
        rclpy.spin_once(self)
        ok = self.sendCmd('GZ_StickyPads', cmd_vals=[1.0,1.0,1.0], cmd_flag=1.0)
        if not ok:
            self.get_logger().error("resetPose failed: StickyPads ON failed")
            return False

        ok = self.step_and_wait(10, tick_timeout=2.0, state_timeout=2.0)
        if not ok:
            self.get_logger().error("resetPose failed: settle after StickyPads ON failed")
            return False

        ok = self.sendCmd('GZ_StickyPads', cmd_vals=[1.0,1.0,1.0], cmd_flag=0.0)
        if not ok:
            self.get_logger().error("resetPose failed: StickyPads OFF failed")
            return False

        ok = self.step_and_wait(10, tick_timeout=2.0, state_timeout=2.0)
        if not ok:
            self.get_logger().error("resetPose failed: settle after StickyPads OFF failed")
            return False
        self.sendCmd('Tumble_Detect',cmd_vals=[1.0,1.0,1.0],cmd_flag=1.0)
        self.sendCmd("Ctrl_Reset", cmd_vals=[1.0,1.0,1.0])
        self.sendCmd("DH_Reset", cmd_vals=[1.0,1.0,1.0])

        ok = self.set_pose_gz([0.0, 0.0, z_0])
        if not ok:
            self.get_logger().error("resetPose failed: temporary pose move failed")
            return False
        
        ok = self.step_and_wait(1000, tick_timeout=15.0, state_timeout=5.0)
        if not ok:
            self.get_logger().error("resetPose failed: first stabilization failed")
            return False

        self.sendCmd('GZ_StickyPads',cmd_vals=[1.0,1.0,1.0],cmd_flag=1.0)

        print("resetPose is completed")

        return True

    def resetPose_Opticalflow(self,z_0=0.4): 
        print("resetPose_Opticalflow is started")


        self.sendCmd('Tumble_Detect',cmd_vals=[1.0,1.0,1.0],cmd_flag=0.0)
        self.sendCmd("Ctrl_Reset", cmd_vals=[1.0,1.0,1.0])
        self.sendCmd("DH_Reset", cmd_vals=[1.0,1.0,1.0])

        # Use subprocess to call the GZ service for setting pose
        # Need to chane "SAR_Config"
        cmd = [
            'gz', 'service', '-s', '/world/empty/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000',
            '--req', f'name: "A15_L250", position: {{x: 0, y: 0, z: {z_0}}}, orientation: {{x: 0, y: 0, z: 0, w: 1}}'
        ]

        result = subprocess.run(cmd, capture_output=True, text=True)

        # print the result of execution
        # if result.returncode == 0:
        #     print("Pose set successfully:", result.stdout)
        # else:
        #     print("Error setting pose:", result.stderr)

        #self._setModelState(pos=[0,0,z_0])
        #self._iterStep(50)
        self._iterStep(1000)
        time.sleep(1)

        self.sendCmd('Tumble_Detect',cmd_vals=[1.0,1.0,1.0],cmd_flag=1.0)
        self.sendCmd("Ctrl_Reset", cmd_vals=[1.0,1.0,1.0])
        self.sendCmd("DH_Reset", cmd_vals=[1.0,1.0,1.0])

# https://github.com/gazebosim/gz-sim/pull/2440
# https://github.com/gazebosim/gz-sim/issues/2318

        # Use subprocess to call the GZ service for setting pose
        # Need to chane "SAR_Config"
        cmd = [
            'gz', 'service', '-s', '/world/empty/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000',
            '--req', f'name: "A15_L250", position: {{x: 0, y: 0, z: {z_0}}}, orientation: {{x: 0, y: 0, z: 0, w: 1}}'
        ]

        result = subprocess.run(cmd, capture_output=True, text=True)

        # print the result of execution
        # if result.returncode == 0:
        #     print("Pose set successfully:", result.stdout)
        # else:
        #     print("Error setting pose:", result.stderr)

        #self._setModelState(pos=[0,0,z_0])
        self._iterStep(500) # Give time for drone to settle
        #rclpy.spin_once(self)
        time.sleep(1)
        
        #self.sendCmd('GZ_StickyPads',cmd_flag=1.0)
        print("resetPose is completed")

    def _setModelState(self,pos=[0,0,0.5],quat=[0,0,0,1],vel=[0,0,0],ang_vel=[0,0,0]):
        print()

    def _setModelInertia(self,Mass=0,Inertia=[0,0,0]):        
        print()
        
        # ## CREATE SERVICE REQUEST MSG
        # srv = Inertia_ParamsRequest() 
        # srv.Mass = Mass
        # srv.Inertia.x = Inertia[0]
        # srv.Inertia.y = Inertia[1]
        # srv.Inertia.z = Inertia[2]

        # ## SEND LOGGING REQUEST VIA SERVICE
        # self.callService("/SAR_Internal/Inertia_Update",srv,Inertia_Params)

    def scaleValue(self,x, original_range=(-1, 1), target_range=(-1, 1)):        
        original_min, original_max = original_range
        target_min, target_max = target_range

        # Scale x to [0, 1] in original range
        x_scaled = (x - original_min) / (original_max - original_min)

        # Scale [0, 1] to target range
        x_target = x_scaled * (target_max - target_min) + target_min
        return x_target

    # ============================
    ##      Command Handlers 
    # ============================
        
    ## ========== GAZEBO FUNCTIONS ==========
    # def handle_Load_Params(self):

    #     print("Reset ROS Parameters\n")
    #     self.loadBaseParams()
    #     self.loadSimParams()
    #     self.sendCmd("Load_Params")
        
    def handle_GZ_StickyPads(self):
        cmd_flag = self.userInput("Turn sticky pads On/Off (1,0): ",float)
        self.sendCmd("GZ_StickyPads",cmd_vals=[1.0,1.0,1.0],cmd_flag=cmd_flag)

    def handle_test_policy(self):
        self.adjustSimSpeed(0.1)
        self.sendCmd("GZ_StickyPads",cmd_vals=[1.0,1.0,1.0],cmd_flag=1.0)
        #self.sendCmd("Optical_Flow_Flag",cmd_vals=[1.0,1.0,1.0],cmd_flag=1.0)

    def handle_test_policy_(self):
        self.sendCmd("Optical_Flow_Flag",cmd_vals=[1.0,1.0,1.0],cmd_flag=0.0)
        self.adjustSimSpeed(1)

    def handle_OpticalFlow_Accuracy(self):

        for V_mag in np.arange(4.0, 4.5, 0.5):  # V_mag: 2.0 ~ 4.0 (Increment by 0.5)
            for V_angle in np.arange(60, 95, 5):  # V_angle: 15 ~ 90 (Increment by 5)
                self.V_mag = V_mag
                self.V_angle = V_angle

                # Add code from DH
                if np.isnan(getattr(self, 'Plane_Angle_rad', np.nan)):
                    self.Plane_Angle_rad = 0
                # Add code from DH
                if np.isnan(getattr(self, 'r_P_O', [np.nan])[0]):
                    self.r_P_O = [3.0, 0.0, 2.5]

                ## CALC STARTING VELOCITY IN GLOBAL COORDS
                V_tx = self.V_mag * np.cos(np.deg2rad(self.V_angle))
                V_perp = self.V_mag * np.sin(np.deg2rad(self.V_angle))
                V_B_P = np.array([V_tx, 0, V_perp])  # {t_x,n_p}
                V_B_O = self.R_PW(V_B_P, self.Plane_Angle_rad)  # {X_W,Z_W}

                EPS = 1e-6  # Epsilon (Prevent division by zero)

                ## CALCULATE STARTING TAU VALUE
                self.Tau_CR_start = 0.75 + np.random.uniform(-0.05, 0.05)

                try:
                    self.Tau_Body_start = (self.Tau_CR_start + self.Collision_Radius / (V_perp + EPS))  # Tau read by body
                except:
                    print("Exception")
                self.Tau_Accel_start = 1  # Acceleration time to desired velocity conditions [s]

                ## CALC STARTING POSITION IN GLOBAL COORDS
                r_P_O = np.array(self.r_P_O)  # Plane Position wrt to Origin - {X_W,Z_W}
                r_P_B = np.array([
                    (self.Tau_CR_start + self.Tau_Accel_start) * V_tx,
                    0,
                    (self.Tau_Body_start + self.Tau_Accel_start) * V_perp
                ])  # Body Position wrt to Plane - {t_x,n_p}
                r_B_O = r_P_O - self.R_PW(r_P_B, self.Plane_Angle_rad)

                ## DESIRED ACCELERATION VALUES Added by DH
                Acc = self.TrajAcc_Max

                a_x = Acc[0]
                a_z = Acc[2]

                ## CALC OFFSET POSITIONS
                t_x = V_B_O[0] / a_x  # Time required to reach Vx
                t_z = V_B_O[2] / a_z  # Time required to reach Vz
                t_total = t_x + t_z + 1

                x_0 = r_B_O[0] - V_B_O[0] ** 2 / (2 * a_x) - V_B_O[0] * t_z  # X-position Vel reached
                y_0 = r_B_O[1]  # Y-position Vel reached
                z_0 = r_B_O[2] - V_B_O[2] ** 2 / (2 * a_z)  # Z-position Vel reached  

                r_B_O = np.array([x_0, y_0, z_0])

                ## LAUNCH QUAD W/ DESIRED VELOCITY
                self.initial_state = (r_B_O, V_B_O)
                self.Sim_VelTraj(pos=r_B_O, vel=V_B_O, t_total=t_total)
                self._iterStep(n_steps=100)

                # Add code from DH : Because of lock step?
                time.sleep(t_total + 8)

                print(f"Initial step is done for V_mag={V_mag}, V_angle={V_angle}\n")
                
                #self._getObs_Tau()
                Tau_DH, Theta_x_DH, Tau_CR, Theta_x, D_perp_CR, Tau_CM = self._getObs_Tau()

#(((((

                # while Tau_CR >= 0.33:
                #     #self._getObs_Tau()
                #     Tau_DH, Theta_x_DH, Tau_CR, Theta_x, D_perp_CR, Tau_CM = self._getObs_Tau()
                #     time.sleep(0.2)
                #     self._iterStep(10)
                #     time.sleep(0.2)

                # print("Start calculating Tau\n")
                # self.adjustSimSpeed(0.05)
                # self.sendCmd("Optical_Flow_Flag", cmd_vals=[1.0, 1.0, 1.0], cmd_flag=1.0)

                # Tau_values = []

                # while Tau_CR >= 0.20:
                #     #self._getObs_Tau()
                #     Tau_DH, Theta_x_DH, Tau_CR, Theta_x, D_perp_CR, Tau_CM = self._getObs_Tau()
                #     time.sleep(0.2)
                #     Tau_values.append([Tau_CR, Tau_DH])
                #     self._iterStep(10)
                #     time.sleep(0.2)
#(((((

#)))))

                while Tau_CM >= 0.55:
                    #self._getObs_Tau()
                    Tau_DH, Theta_x_DH, Tau_CR, Theta_x, D_perp_CR, Tau_CM = self._getObs_Tau()
                    time.sleep(0.1)
                    self._iterStep(10)
                    time.sleep(0.1)


                print("Start calculating Tau\n")
                self.adjustSimSpeed(0.05)
                self.sendCmd("Optical_Flow_Flag", cmd_vals=[1.0, 1.0, 1.0], cmd_flag=1.0)

                Tau_values = []

                while Tau_CM >= 0.05:
                    print("Tau_CM : ",Tau_CM)
                    #self._getObs_Tau()
                    Tau_DH, Theta_x_DH, Tau_CR, Theta_x, D_perp_CR, Tau_CM = self._getObs_Tau()
                    time.sleep(0.1)
                    Tau_values.append([Tau_CM, Tau_DH])
                    self._iterStep(10)
                    time.sleep(0.1)
#))))))

                # 파일 이름을 V_mag과 V_angle을 포함하도록 설정
                fileName = f"OFA[{V_mag:.2f},{V_angle:.2f}].csv"
                filePath = os.path.join(self.Log_Dir, fileName)

#(((((

                # with open(filePath, 'w', newline='') as file:
                #     writer = csv.writer(file, delimiter=',')
                #     writer.writerow(["Tau_CR", "Tau_DH"])
                #     writer.writerows(Tau_values)

#(((((

#)))))
                with open(filePath, 'w', newline='') as file:
                    writer = csv.writer(file, delimiter=',')
                    writer.writerow(["Tau_CM", "Tau_DH"])
                    writer.writerows(Tau_values)
#)))))
                self.sendCmd("Optical_Flow_Flag", cmd_vals=[1.0, 1.0, 1.0], cmd_flag=0.0)
                self.adjustSimSpeed(1.0)


                self.resetPose_Opticalflow()

                print(f"handle_OpticalFlow_Accuracy is done for V_mag={V_mag}, V_angle={V_angle}\n")
                
    # def handle_OpticalFlow_Accuracy(self):

    #     ## GET RELATIVE VEL CONDITIONS 
    #     self.V_mag, self.V_angle = self.userInput("Flight Velocity (V_mag,V_angle):",float)

    #     # Add code from DH
    #     if np.isnan(getattr(self, 'Plane_Angle_rad', np.nan)):
    #         self.Plane_Angle_rad = 0
    #     # Add code from DH
    #     if np.isnan(getattr(self, 'r_P_O', [np.nan])[0]):
    #         self.r_P_O = [3.0, 0.0, 2.5]

    #     ## CALC STARTING VELOCITY IN GLOBAL COORDS
    #     V_tx = self.V_mag*np.cos(np.deg2rad(self.V_angle))
    #     V_perp = self.V_mag*np.sin(np.deg2rad(self.V_angle))
    #     V_B_P = np.array([V_tx,0,V_perp])               # {t_x,n_p}
    #     V_B_O = self.R_PW(V_B_P,self.Plane_Angle_rad)   # {X_W,Z_W}

    #     EPS = 1e-6 # Epsilon (Prevent division by zero)

    #     ## CALCULATE STARTING TAU VALUE
    #     # self.Tau_CR_start = self.t_rot_max*np.random.uniform(0.9,1.1) # Add noise to starting condition
        
    #     #self.Tau_CR_start = 0.5 + np.random.uniform(-0.05,0.05)
    #     self.Tau_CR_start = 0.75 + np.random.uniform(-0.05,0.05)
        
    #     try:
    #         self.Tau_Body_start = (self.Tau_CR_start + self.Collision_Radius/(V_perp+EPS)) # Tau read by body
    #     except:
    #         print("Exception")
    #     self.Tau_Accel_start = 1 # Acceleration time to desired velocity conditions [s]

    #     ## CALC STARTING POSITION IN GLOBAL COORDS
    #     # (Derivation: Research_Notes_Book_3.pdf (9/17/23))

    #     r_P_O = np.array(self.r_P_O)                                        # Plane Position wrt to Origin - {X_W,Z_W}
    #     r_P_B = np.array([(self.Tau_CR_start + self.Tau_Accel_start)*V_tx,
    #                       0,
    #                       (self.Tau_Body_start + self.Tau_Accel_start)*V_perp])  # Body Position wrt to Plane - {t_x,n_p}
    #     r_B_O = r_P_O - self.R_PW(r_P_B,self.Plane_Angle_rad)    
    #     #print("r_P_B : ", r_P_B)

    #     ## DESIRED ACCELERATION VALUES Added by DH
    #     Acc = self.TrajAcc_Max
    
    #     a_x = Acc[0]
    #     a_z = Acc[2]

    #     ## CALC OFFSET POSITIONS
    #     t_x = V_B_O[0]/a_x    # Time required to reach Vx
    #     t_z = V_B_O[2]/a_z    # Time required to reach Vz
    #     t_total = t_x + t_z + 1

    #     x_0 = r_B_O[0] - V_B_O[0]**2/(2*a_x) - V_B_O[0]*t_z     # X-position Vel reached
    #     y_0 = r_B_O[1]                                          # Y-position Vel reached
    #     z_0 = r_B_O[2] - V_B_O[2]**2/(2*a_z)                    # Z-position Vel reached  

    #     r_B_O = np.array([x_0,y_0,z_0])

    #     ## LAUNCH QUAD W/ DESIRED VELOCITY
    #     self.initial_state = (r_B_O,V_B_O)
    #     #print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& r_B_O :",r_B_O)
    #     #print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& V_B_O :",V_B_O)
    #     self.Sim_VelTraj(pos=r_B_O,vel=V_B_O,t_total=t_total)
    #     self._iterStep(n_steps=100)
        
    #     # Add code from DH : Beacause of lock step?
    #     time.sleep(t_total+8)

    #     print("Initial step is done\n")

    #     self._getObs_Tau()
    #     Tau_DH, Theta_x_DH, Tau_CR, Theta_x, D_perp_CR = self._getObs_Tau()

    #     while(Tau_CR >= 0.35):
    #         #rclpy.spin_once(self)
    #         self._getObs_Tau()
    #         Tau_DH, Theta_x_DH, Tau_CR, Theta_x, D_perp_CR = self._getObs_Tau()
    #         self._iterStep(10)

    #     print("start calculating Tau\n")
    #     self.adjustSimSpeed(0.05)
    #     self.sendCmd("Optical_Flow_Flag",cmd_vals=[1.0,1.0,1.0],cmd_flag=1.0)

    #     Tau_values = []

    #     while(Tau_CR >= 0.15):
    #         #rclpy.spin_once(self)
    #         self._getObs_Tau()
    #         Tau_DH, Theta_x_DH, Tau_CR, Theta_x, D_perp_CR = self._getObs_Tau()
    #         Tau_values.append([Tau_CR, Tau_DH]) 
    #         self._iterStep(10)

    #     fileName = fileName if 'fileName' in locals() else "OpticalflowAccuracy.csv"
    #     filePath = os.path.join(self.Log_Dir,fileName)

    #     with open(filePath, 'w', newline='') as file:
    #         writer = csv.writer(file, delimiter=',')
    #         writer.writerow(["Tau_CR", "Tau_DH"])
    #         writer.writerows(Tau_values)


    #     print("handle_OpticalFlow_Accuracy is done\n")


    # def handle_GZ_Pose_Reset(self):
    #     print("Reset Pos/Vel -- Sticky off -- Controller Reset\n")
    #     self.resetPose()
    #     self.pausePhysics(pause_flag=False)

    def handle_GZ_Global_Vel_traj(self):

        # GET GLOBAL VEL CONDITIONS 
        V_mag,V_angle = self.userInput("Flight Velocity (V_mag,V_angle):",float)

        # CALC GLOBAL VELOCITIES
        Vx = V_mag*np.cos(np.radians(V_angle))
        Vy = 0
        Vz = V_mag*np.sin(np.radians(V_angle))
        V_B_O = [Vx,Vy,Vz]

        self.Sim_VelTraj(self.r_B_O,V_B_O)
        self.pausePhysics(False)

    # def handle_GZ_Rel_Vel_traj(self):

    #     # GET RELATIVE VEL CONDITIONS 
    #     V_mag,V_angle = self.userInput("Flight Velocity (V_mag,V_angle):",float)

    #     # CALC RELATIVE VELOCITIES
    #     V_tx = V_mag*np.cos(np.radians(V_angle))
    #     V_ty = 0
    #     V_perp = V_mag*np.sin(np.radians(V_angle))

    #     # CALCULATE GLOBAL VELOCITIES
    #     V_B_O = self.R_PW(np.array([V_tx,V_ty,V_perp]),self.Plane_Angle_rad)

    #     self.Sim_VelTraj(self.r_B_O,V_B_O)
    #     self.pausePhysics(False)

    # ================================
    ##     SIM MONITORING/LAUNCH
    # ================================
    def _launch_GZ_Sim(self):
        cmd = "gnome-terminal --disable-factory  --geometry 85x46+1050+0 -- ros2 launch sar_launch Gazebo_sim.launch.py"
        self.GZ_Sim_process = subprocess.Popen(cmd, shell=True)

    def _launch_SAR_DC(self):
        cmd = "gnome-terminal --disable-factory --geometry 85x46+1050+0 -- ros2 run sar_data_converter SAR_DataConverter"
        self.SAR_DC_process = subprocess.Popen(cmd, shell=True)

    def _launch_Controller(self):
        cmd = "gnome-terminal --disable-factory --geometry 85x46+1050+0 -- ros2 run sar_control SAR_Controller"
        self.SAR_Ctrl_process = subprocess.Popen(cmd, shell=True)
        
    def _start_monitoring_subprocesses(self):
        if self.monitor_thread is not None and self.monitor_thread.is_alive():
            return

        self.monitor_thread = Thread(target=self._monitor_subprocesses)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()

    def _monitor_subprocesses(self):

        while True:

            #self.GZ_ping_ok = self._ping_service("/SAR_DC/CMD_Input",timeout=10)
            
            time.sleep(0.5)

    def _wait_for_sim_running(self, timeout=20.0, interval=0.1):
        start_time = time.time()

        cmd_client = self.getServiceClient(CTRLCmdSrv, '/SAR_DC/CMD_Input', creation_timeout=3.0)
        obs_client = self.getServiceClient(CTRLGetObs, '/CTRL/Get_Obs', creation_timeout=3.0)

        clock_seen = False
        cmd_ready = False
        obs_ready = False

        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)

            if self.clock_msg_count > 0 and not np.isnan(self.t):
                clock_seen = True

            if cmd_client.service_is_ready():
                cmd_ready = True

            if obs_client.service_is_ready():
                obs_ready = True

            if clock_seen and cmd_ready and obs_ready:
                self.get_logger().info(
                    f"Simulation ready: clock={clock_seen}, "
                    f"CMD_Input={cmd_ready}, CTRL_Get_Obs={obs_ready}"
                )
                return True

            time.sleep(interval)

        self.get_logger().error(
            f"Timeout waiting for simulation ready state. "
            f"clock={clock_seen}, CMD_Input={cmd_ready}, CTRL_Get_Obs={obs_ready}"
        )
        return False

    def _ping_service(self, service_name, timeout=5, silence_errors=False):
        cmd = f"ros2 service call {service_name} sar_msgs/srv/CTRLCmdSrv '{{}}'"
        stderr_option = subprocess.DEVNULL if silence_errors else None

        try:
            result = subprocess.run(cmd, shell=True, timeout=timeout, check=True, stdout=subprocess.PIPE, stderr=stderr_option)
            
            #print(f"Command Output: {result.stdout.decode('utf-8')}")

            return result.returncode == 0
        except subprocess.TimeoutExpired:
            #print("Timeout expired while calling the service.")
            return False
        except subprocess.CalledProcessError:
            #print("Error occurred while calling the service.")
            return False
        
    def _kill_Sim(self):
        try:
            subprocess.run(['pkill', '-f', 'gz sim'], check=True)
        except subprocess.CalledProcessError:
            self.get_logger().warn("No gz sim process found to kill.")

        try:
            subprocess.run(['pkill', '-f', 'SAR_DataConverter'], check=True)
        except subprocess.CalledProcessError:
            self.get_logger().warn("No SAR_DataConverter process found to kill.")

        time.sleep(2)

    def _clear_sim_runtime_state(self):
        """Clear cached sim/runtime values before a fresh restart."""
        self.Clock_Check_Flag.clear()

        self.t = np.nan
        self.clock_msg_count = 0
        self.state_msg_count = 0

        self.r_B_O = np.array([np.nan, np.nan, np.nan], dtype=float)
        self.V_B_O = np.array([np.nan, np.nan, np.nan], dtype=float)

        self.Trg_Flag = False
        self.Impact_Flag_Ext = False
        self.Done = False

    def _nudge_sim_clock(self):
        try:
            self.pausePhysics(False)
            time.sleep(0.2)
            self.pausePhysics(True)
        except Exception as e:
            self.get_logger().warn(f"_nudge_sim_clock failed: {e}")

    def _restart_sim_safely(self, ready_timeout=25.0):
        with self.sim_restart_lock:
            if self.is_restarting_sim:
                self.get_logger().warn("Simulation restart already in progress.")
                return False

            self.is_restarting_sim = True
            try:
                self.get_logger().info("Starting safe simulation restart")

                self._kill_Sim()
                time.sleep(1.0)

                self._clear_sim_runtime_state()
                self._restart_Sim()
                self._start_monitoring_subprocesses()

                self._nudge_sim_clock()
                ok = self._wait_for_sim_fully_ready(timeout=ready_timeout)

                if not ok:
                    self.get_logger().warn("First ready check failed, retrying clock nudge once...")
                    self._nudge_sim_clock()
                    ok = self._wait_for_sim_fully_ready(timeout=10.0)

                if ok:
                    self.get_logger().info("Safe simulation restart completed successfully")
                else:
                    self.get_logger().error("Safe simulation restart failed")
                return ok

            finally:
                self.is_restarting_sim = False

    def _restart_Sim(self):
        self.Clock_Check_Flag.clear()

        self._launch_GZ_Sim()
        self._launch_SAR_DC()

        if not self._wait_for_node(node_name="SAR_DataConverter_Node", timeout=10, interval=0.25):
            raise RuntimeError("SAR_DataConverter_Node failed to launch.")

        # launch 완료 후 clock 체크 허용
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

    def _wait_for_sim_fully_ready(self, timeout=20.0, interval=0.05,
                                min_new_clock_msgs=2, min_new_state_msgs=2):
        start_wall = time.time()

        start_clock_count = self.clock_msg_count
        start_state_count = self.state_msg_count

        cmd_client = self.getServiceClient(CTRLCmdSrv, '/SAR_DC/CMD_Input', creation_timeout=3.0)
        obs_client = self.getServiceClient(CTRLGetObs, '/CTRL/Get_Obs', creation_timeout=3.0)

        clock_ready = False
        state_ready = False
        cmd_ready = False
        obs_ready = False

        while time.time() - start_wall < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)

            # 핵심 수정:
            # restart 직후에는 sim_time 증가 여부보다
            # "새로운 /clock 메시지가 들어왔는가"를 우선 본다.
            clock_ready = (
                self.clock_msg_count >= start_clock_count + min_new_clock_msgs
                and not np.isnan(self.t)
            )

            state_ready = (
                self.state_msg_count >= start_state_count + min_new_state_msgs
                and not np.isnan(self.r_B_O[0])
                and not np.isnan(self.V_B_O[0])
            )

            cmd_ready = cmd_client.service_is_ready()
            obs_ready = obs_client.service_is_ready()

            if clock_ready and state_ready and cmd_ready and obs_ready:
                self.get_logger().info(
                    f"Sim fully ready: clock_ready={clock_ready}, "
                    f"state_ready={state_ready}, cmd_ready={cmd_ready}, "
                    f"obs_ready={obs_ready}, sim_time={self.t:.3f}"
                )
                return True

            time.sleep(interval)

        sim_time_str = "nan" if np.isnan(self.t) else f"{self.t:.3f}"
        self.get_logger().error(
            f"Sim fully ready timeout: clock_ready={clock_ready}, "
            f"state_ready={state_ready}, cmd_ready={cmd_ready}, "
            f"obs_ready={obs_ready}, sim_time={sim_time_str}"
        )
        return False

    def _wait_for_post_reset_ready(self, timeout=8.0, interval=0.05, min_new_state_msgs=2):
        start_wall = time.time()
        start_state_count = self.state_msg_count
        start_clock_count = self.clock_msg_count

        while time.time() - start_wall < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)

            state_ready = (
                self.state_msg_count >= start_state_count + min_new_state_msgs
                and not np.isnan(self.r_B_O[0])
                and not np.isnan(self.V_B_O[0])
            )

            clock_ready = self.clock_msg_count > start_clock_count

            if state_ready and clock_ready:
                self.get_logger().info(
                    f"Post-reset ready: state_ready={state_ready}, "
                    f"clock_ready={clock_ready}"
                )
                return True

            time.sleep(interval)

        self.get_logger().warn(
            f"Post-reset readiness timed out. "
            f"new_state_msgs={self.state_msg_count - start_state_count}, "
            f"new_clock_msgs={self.clock_msg_count - start_clock_count}"
        )
        return False

    def getServiceClient(self, srv_type, srv_addr, creation_timeout=3.0):
        """
        Return cached service client.
        If client does not exist, create it once and wait for service only once.
        """
        key = (srv_addr, srv_type)

        if key not in self.service_clients:
            self.get_logger().info(f"Creating new service client for '{srv_addr}'")
            client = self.create_client(srv_type, srv_addr)

            if not client.wait_for_service(timeout_sec=creation_timeout):
                self.get_logger().error(
                    f"Service '{srv_addr}' not available at creation "
                    f"(waited {creation_timeout:.1f}s)."
                )
                # 그래도 cache에는 넣어둔다.
                # 나중에 service가 살아나면 재사용 가능함.
            else:
                self.get_logger().info(
                    f"Service '{srv_addr}' is available and client is ready."
                )

            self.service_clients[key] = client

        return self.service_clients[key]

    def wait_for_fresh_state(self, min_state_inc=1, min_clock_inc=1, timeout_wall_sec=3.0):
        """
        Wait until fresh state/clock callbacks arrive.

        Returns:
            bool: True if enough fresh callbacks arrived before timeout.
        """
        state0 = getattr(self, 'state_msg_count', 0)
        clock0 = getattr(self, 'clock_msg_count', 0)

        start_wall = time.time()
        while time.time() - start_wall < timeout_wall_sec:
            try:
                rclpy.spin_once(self, timeout_sec=0.02)
            except Exception as e:
                self.get_logger().warn(f"wait_for_fresh_state spin_once failed: {e}")

            state_ok = (getattr(self, 'state_msg_count', 0) - state0) >= min_state_inc
            clock_ok = (getattr(self, 'clock_msg_count', 0) - clock0) >= min_clock_inc

            if state_ok and clock_ok:
                return True

        self.get_logger().warn(
            f"wait_for_fresh_state timeout: "
            f"state_delta={getattr(self, 'state_msg_count', 0) - state0}, "
            f"clock_delta={getattr(self, 'clock_msg_count', 0) - clock0}"
        )
        return False

    def wait_for_tick_advance(self, tick_start, expected_delta, timeout_wall_sec=5.0):
        target_tick = tick_start + expected_delta

        # Tick slack policy:
        # - very small steps (<10): strict
        # - 10 steps: allow up to 5 ticks lag
        # - 1000 steps: allow up to 100 ticks lag
        if expected_delta < 10:
            tick_slack = 0
        else:
            tick_slack = int(round(expected_delta * 0.10))
            tick_slack = max(5, tick_slack)
            tick_slack = min(100, tick_slack)

        start_wall = time.time()

        while time.time() - start_wall < timeout_wall_sec:
            try:
                rclpy.spin_once(self, timeout_sec=0.02)
            except Exception as e:
                self.get_logger().warn(f"wait_for_tick_advance spin_once failed: {e}")

            try:
                cur_tick = self._getTick()
            except Exception as e:
                self.get_logger().warn(f"wait_for_tick_advance: _getTick failed: {e}")
                time.sleep(0.05)
                continue

            if cur_tick >= target_tick - tick_slack:
                if cur_tick < target_tick:
                    self.get_logger().warn(
                        f"Tick advanced with allowed lag: "
                        f"expected_delta={expected_delta}, "
                        f"target={target_tick}, "
                        f"current={cur_tick}, "
                        f"slack={tick_slack}"
                    )
                return True

        try:
            cur_tick = self._getTick()
        except Exception:
            cur_tick = "unavailable"

        self.get_logger().warn(
            f"wait_for_tick_advance timeout: "
            f"expected_delta={expected_delta}, "
            f"target={target_tick}, "
            f"current={cur_tick}, "
            f"slack={tick_slack}"
        )
        return False

    # def wait_for_tick_advance(self, tick_start, expected_delta, timeout_wall_sec=5.0):
    #     target_tick = tick_start + expected_delta
    #     start_wall = time.time()

    #     while time.time() - start_wall < timeout_wall_sec:
    #         try:
    #             rclpy.spin_once(self, timeout_sec=0.02)
    #         except Exception as e:
    #             self.get_logger().warn(f"wait_for_tick_advance spin_once failed: {e}")

    #         try:
    #             cur_tick = self._getTick()
    #         except Exception as e:
    #             self.get_logger().warn(f"wait_for_tick_advance: _getTick failed: {e}")
    #             time.sleep(0.05)
    #             continue

    #         if cur_tick >= target_tick:
    #             return True

    #     try:
    #         cur_tick = self._getTick()
    #     except Exception:
    #         cur_tick = "unavailable"

    #     self.get_logger().warn(
    #         f"wait_for_tick_advance timeout: target={target_tick}, current={cur_tick}"
    #     )
    #     return False

    def step_and_wait(self, n_steps, tick_timeout=5.0, state_timeout=3.0, require_fresh_state=True):
        """
        Execute sim steps and wait until tick/state are updated.

        Args:
            n_steps (int): number of sim steps
            tick_timeout (float): wall-clock timeout for tick advance
            state_timeout (float): wall-clock timeout for fresh state
            require_fresh_state (bool): whether to require fresh state callbacks

        Returns:
            bool
        """
        try:
            tick0 = self._getTick()
        except Exception as e:
            self.get_logger().error(f"step_and_wait failed to read initial tick: {e}")
            return False

        ok_step = self._iterStep(n_steps=n_steps)
        if not ok_step:
            self.get_logger().error(f"step_and_wait failed: _iterStep failed for {n_steps} steps")
            return False

        ok_tick = self.wait_for_tick_advance(
            tick_start=tick0,
            expected_delta=n_steps,
            timeout_wall_sec=tick_timeout
        )
        if not ok_tick:
            self.get_logger().error(f"step_and_wait failed: tick did not advance by {n_steps}")
            return False

        if require_fresh_state:
            ok_state = self.wait_for_fresh_state(timeout_wall_sec=state_timeout)
            if not ok_state:
                self.get_logger().error("step_and_wait failed: no fresh state/clock received")
                return False

        return True

    def callService(self, srv_addr, srv_msg, srv_type, num_retries=3, call_timeout=2.0):
        """
        Call a ROS2 service using a cached client.

        Important:
        - While waiting for the async future, actively spin this node.
        - This prevents timeouts caused by the future never being serviced.
        """
        client = self.getServiceClient(srv_type, srv_addr)

        if not client.service_is_ready():
            self.get_logger().warn(
                f"Service '{srv_addr}' is not ready at call time."
            )
            return None

        for retry in range(num_retries):
            try:
                future = client.call_async(srv_msg)
            except Exception as e:
                self.get_logger().warn(
                    f"Failed to send async request to '{srv_addr}' "
                    f"({retry+1}/{num_retries}): {e}"
                )
                continue

            start_time = time.time()

            while time.time() - start_time < call_timeout:
                # 핵심: future completion을 위해 spin_once 수행
                try:
                    rclpy.spin_once(self, timeout_sec=0.05)
                except Exception as e:
                    self.get_logger().warn(
                        f"spin_once failed while waiting for service '{srv_addr}' "
                        f"({retry+1}/{num_retries}): {e}"
                    )
                    break

                if future.done():
                    try:
                        return future.result()
                    except Exception as e:
                        self.get_logger().warn(
                            f"Service call failed for '{srv_addr}' "
                            f"({retry+1}/{num_retries}): {e}"
                        )
                        break

            self.get_logger().warn(
                f"Timeout waiting for service '{srv_addr}' response "
                f"({retry+1}/{num_retries})"
            )

        self.get_logger().warn(
            f"Service call to '{srv_addr}' failed after {num_retries} retries."
        )
        return None


    def pausePhysics(self, pause_flag=True):
        if pause_flag:
            cmd = [
                'gz', 'service', '-s', '/world/empty/control',
                '--reqtype', 'gz.msgs.WorldControl',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '3000',
                '--req', 'pause: true'
            ]
        else:
            cmd = [
                'gz', 'service', '-s', '/world/empty/control',
                '--reqtype', 'gz.msgs.WorldControl',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '3000',
                '--req', 'pause: false'
            ]

        result = subprocess.run(cmd, capture_output=True, text=True)
        return result.returncode == 0


    # gz service -s /world/empty/set_physics --reqtype gz.msgs.Physics --reptype gz.msgs.Boolean --timeout 3000 --req 'real_time_factor: 0.2'
    def adjustSimSpeed(self,Real_time_factor=1.0):

        cmd = f"gz service -s /world/empty/set_physics --reqtype gz.msgs.Physics --reptype gz.msgs.Boolean --timeout 3000 --req 'real_time_factor: {Real_time_factor}'"
        self.adjust_simulation_speed_process = subprocess.Popen(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def spin(self):
        rclpy.spin(self.node)
    
    def destroy(self):
        self.node.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    env = SAR_Sim_Interface()

    try:
        env.initialize_sim()
        env.spin()
    except KeyboardInterrupt:
        pass
    finally:
        env.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    