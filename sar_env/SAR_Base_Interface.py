#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from ament_index_python.packages import get_package_share_directory

import os
import numpy as np
import getpass
import asyncio
import yaml
import sys

import asyncio #For Debug

# ROS2 messages
from sar_msgs.msg import SARStateData
from sar_msgs.msg import SARTriggerData
from sar_msgs.msg import SARImpactData
from sar_msgs.msg import SARMiscData
from sar_msgs.msg import ROSParams

from sar_msgs.srv import CTRLCmdSrv
from sar_msgs.srv import LoggingCMD

YELLOW = '\033[93m'
RED = '\033[91m'
GREEN = '\033[92m'
BLUE = "\033[34m"  # Blue text
RESET = "\033[0m"  # Reset to default color

class SAR_Base_Interface(Node):

    def __init__(self,Experiment_Setup=False):
        
        ## LOGGING PARAMETERS
        sys.path.append('/home/dlee/ros2_ws/src/sar_simulation')
        self.BASE_PATH = '/home/dlee/ros2_ws/src/sar_simulation'
        self.Log_Dir = f"{self.BASE_PATH}/sar_logging/local_logs"
        self.Log_Name = "TestLog.csv"
        self.Error_Str = "No_Debug_Data"

        # ## LOGGING PARAMETERS
        # self.Username = getpass.getuser()
        # self.Log_Dir = f"/home/{self.Username}/ros2_ws/src/sar_simulation/sar_logging/local_logs"
        # self.Log_Name = "TestLog.csv"
        # self.Error_Str = "No_Debug_Data"

        if not Experiment_Setup:  
            super().__init__('SAR_Env_Node')
            self.loadBaseParams()
            self._preInit()

        else:
            self.loadBaseParams()
            self._preInit()

        print(f"{GREEN}")
        print("=============================================")
        print("       SAR Base Interface Initialized        ")
        print("=============================================")

        print(f"SAR Type: {self.SAR_Type} -- SAR Config: {self.SAR_Config}\n")
        print(f"Leg Length: {self.Leg_Length:.3f} m \t Leg Angle: {self.Leg_Angle:.3f} deg")
        print(f"L_eff: {self.L_eff:.3f} m \t\t Gamma_eff: {self.Gamma_eff:.1f} deg\n")
        print(f"Phi_impact_P_B_Min: {self.Phi_P_B_impact_Min_deg:.1f} deg\n")

        print(f"Thrust_max: {self.Thrust_max:.0f} g")
        print(f"Ang_Acc_Max: {self.Ang_Acc_max:.0f} rad/s^2")
        print(f"TrajAcc_Max: [{self.TrajAcc_Max[0]:.1f}, {self.TrajAcc_Max[1]:.1f}, {self.TrajAcc_Max[2]:.1f}]  m/s^2") 
        print(f"{RESET}")

        ## SAR DATA SUBSCRIBERS 
        # NOTE: Queue sizes=1 so that we are always looking at the most current data and 
        #       not data at back of a queue waiting to be processed by callbacks
        #self.r_B_O = [0,0,0]
        self.StateData_subscriber = self.create_subscription(SARStateData,'/SAR_DC/StateData',self._SAR_StateDataCallback,1)
        self.TriggerData_subscriber = self.create_subscription(SARTriggerData,'/SAR_DC/TriggerData',self._SAR_TriggerDataCallback,1)
        self.ImpactData_subscriber = self.create_subscription(SARImpactData,'/SAR_DC/ImpactData',self._SAR_ImpactDataCallback,1)
        self.MiscData_subscriber = self.create_subscription(SARMiscData,'/SAR_DC/MiscData',self._SAR_MiscDataCallback,1)
        
        self.ROSParams_subscriber = self.create_subscription(ROSParams,'/ROS2/PARAMETER',self._ROS_PARAMETERCallback,1)

        # !!!!! Need to Add
        # ## RL DATA PUBLISHERS
        # self.RL_Data_Pub = rospy.Publisher("/RL/Data",RL_Data,queue_size=10)
        # self.RL_History_Pub = rospy.Publisher("/RL/History",RL_History,queue_size=10)

    def loadBaseParams(self):
        # LOAD BASE PARAMETERS
        param_path_list = [
            f"{self.BASE_PATH}/sar_config/Base_Settings_Bryan.yaml",
            f"{self.BASE_PATH}/sar_config/Model_Types_Bryan.yaml",
            f"{self.BASE_PATH}/sar_config/Cam_Types_Bryan.yaml",
        ]
        
        for path in param_path_list:
            with open(path, 'r') as file:
                loaded_parameters = yaml.safe_load(file)

            self.declare_parameters_from_dict(loaded_parameters)
        
        ## SAR PARAMETERS
        self.SAR_Type = self.get_parameter('SAR_SETTINGS.SAR_Type').get_parameter_value().string_value
        self.SAR_Config = self.get_parameter('SAR_SETTINGS.SAR_Config').get_parameter_value().string_value
        self.Policy_Type = self.get_parameter('SAR_SETTINGS.Policy_Type').get_parameter_value().string_value
        # self.get_logger().info(f'SAR_Type: {self.SAR_Type}')
        # self.get_logger().info(f'SAR_Config: {self.SAR_Config}')
        # self.get_logger().info(f'Policy_Type: {self.Policy_Type}')

        ## INERTIAL PARAMETERS
        self.Ref_Mass = self.get_parameter(f"SAR_Type.{self.SAR_Type}.Config.{self.SAR_Config}.Ref_Mass").get_parameter_value().double_value
        self.Ref_Ixx = self.get_parameter(f"SAR_Type.{self.SAR_Type}.Config.{self.SAR_Config}.Ref_Ixx").get_parameter_value().double_value
        self.Ref_Iyy = self.get_parameter(f"SAR_Type.{self.SAR_Type}.Config.{self.SAR_Config}.Ref_Iyy").get_parameter_value().double_value
        self.Ref_Izz = self.get_parameter(f"SAR_Type.{self.SAR_Type}.Config.{self.SAR_Config}.Ref_Izz").get_parameter_value().double_value
        
        # self.get_logger().info(f'Ref_Mass: {self.Ref_Mass}')
        # self.get_logger().info(f'Ref_Ixx: {self.Ref_Ixx}')
        # self.get_logger().info(f'Ref_Iyy: {self.Ref_Iyy}')
        # self.get_logger().info(f'Ref_Izz: {self.Ref_Izz}')

        self.Base_Mass = self.get_parameter(f"SAR_Type.{self.SAR_Type}.System_Params.Base_Mass").get_parameter_value().double_value
        self.Base_Ixx = self.get_parameter(f"SAR_Type.{self.SAR_Type}.System_Params.Base_Ixx").get_parameter_value().double_value
        self.Base_Iyy = self.get_parameter(f"SAR_Type.{self.SAR_Type}.System_Params.Base_Iyy").get_parameter_value().double_value
        self.Base_Izz = self.get_parameter(f"SAR_Type.{self.SAR_Type}.System_Params.Base_Izz").get_parameter_value().double_value
        
        # self.get_logger().info(f'Base_Mass: {self.Base_Mass}')
        # self.get_logger().info(f'Base_Ixx: {self.Base_Ixx}')
        # self.get_logger().info(f'Base_Iyy: {self.Base_Iyy}')
        # self.get_logger().info(f'Base_Izz: {self.Base_Izz}')

        ## GEOMETRIC PARAMETERS
        self.Forward_Reach = self.get_parameter(f"SAR_Type.{self.SAR_Type}.System_Params.Forward_Reach").get_parameter_value().double_value
        self.Leg_Length = self.get_parameter(f"SAR_Type.{self.SAR_Type}.Config.{self.SAR_Config}.Leg_Length").get_parameter_value().double_value
        self.Leg_Angle = self.get_parameter(f"SAR_Type.{self.SAR_Type}.Config.{self.SAR_Config}.Leg_Angle").get_parameter_value().double_value
        self.Prop_Front = self.get_parameter(f"SAR_Type.{self.SAR_Type}.System_Params.Prop_Front").get_parameter_value().double_array_value
        self.Prop_Rear = self.get_parameter(f"SAR_Type.{self.SAR_Type}.System_Params.Prop_Rear").get_parameter_value().double_array_value

        # self.get_logger().info(f'Forward_Reach: {self.Forward_Reach}')
        # self.get_logger().info(f'Leg_Length: {self.Leg_Length}')
        # self.get_logger().info(f'Leg_Angle: {self.Leg_Angle}')
        # self.get_logger().info(f'Prop_Front: {self.Prop_Front}')
        # self.get_logger().info(f'Prop_Rear: {self.Prop_Rear}')

        ## EFFECTIVE-GEOEMTRIC PARAMETERS
        self.L_eff = self.get_parameter(f"SAR_Type.{self.SAR_Type}.Config.{self.SAR_Config}.L_eff").get_parameter_value().double_value
        self.Gamma_eff = self.get_parameter(f"SAR_Type.{self.SAR_Type}.Config.{self.SAR_Config}.Gamma_eff").get_parameter_value().double_value
        self.Lx_eff = self.L_eff*np.sin(np.radians(self.Gamma_eff))
        self.Lz_eff = self.L_eff*np.cos(np.radians(self.Gamma_eff))
        self.Collision_Radius = self.L_eff

        # self.get_logger().info(f'L_eff: {self.L_eff}')
        # self.get_logger().info(f'Gamma_eff: {self.Gamma_eff}')

        ## SYSTEM AND FLIGHT PARAMETERS
        self.TrajAcc_Max = self.get_parameter(f"SAR_Type.{self.SAR_Type}.System_Params.TrajAcc_Max").get_parameter_value().double_array_value
        self.TrajJerk_Max = self.get_parameter(f"SAR_Type.{self.SAR_Type}.System_Params.TrajJerk_Max").get_parameter_value().double_array_value
        self.Tau_up = self.get_parameter(f"SAR_Type.{self.SAR_Type}.System_Params.Tau_up").get_parameter_value().double_value
        self.Tau_down = self.get_parameter(f"SAR_Type.{self.SAR_Type}.System_Params.Tau_down").get_parameter_value().double_value
        self.Thrust_max = self.get_parameter(f"SAR_Type.{self.SAR_Type}.System_Params.Thrust_max").get_parameter_value().double_value
        self.C_tf = self.get_parameter(f"SAR_Type.{self.SAR_Type}.System_Params.C_tf").get_parameter_value().double_value
        self.Ang_Acc_max = (9.81*self.Thrust_max*1e-3*self.Prop_Front[0])*2/self.Ref_Iyy
        self.setAngAcc_range([-self.Ang_Acc_max, self.Ang_Acc_max])
        
        self.Beta_Min_deg = -(self.Gamma_eff + np.degrees(np.arctan2(self.Forward_Reach-self.Lx_eff,self.Lz_eff)))
        self.Phi_P_B_impact_Min_deg = -self.Beta_Min_deg - self.Gamma_eff + 90

        # self.get_logger().info(f'TrajAcc_Max: {self.TrajAcc_Max}')
        # self.get_logger().info(f'TrajJerk_Max: {self.TrajJerk_Max}')
        # self.get_logger().info(f'Tau_up: {self.Tau_up}')
        # self.get_logger().info(f'Tau_down: {self.Tau_down}')
        # self.get_logger().info(f'Thrust_max: {self.Thrust_max}')
        # self.get_logger().info(f'C_tf: {self.C_tf}')

        ## CAM PARAMETERS
        self.Cam_Config = self.get_parameter(f"CAM_SETTINGS.Cam_Config").get_parameter_value().string_value
        self.Cam_Active = self.get_parameter(f"CAM_SETTINGS.Cam_Active").get_parameter_value().bool_value
        
        # self.get_logger().info(f'Cam_Config: {self.Cam_Config}')
        # self.get_logger().info(f'Cam_Active: {self.Cam_Active}')

        ## PLANE PARAMETERS
        self.Plane_Type = self.get_parameter(f"PLANE_SETTINGS.Plane_Type").get_parameter_value().string_value
        self.Plane_Config = self.get_parameter(f"PLANE_SETTINGS.Plane_Config").get_parameter_value().string_value
        self.Plane_Pos_x_init = self.get_parameter(f"PLANE_SETTINGS.Pos_X_init").get_parameter_value().double_value
        self.Plane_Pos_y_init = self.get_parameter(f"PLANE_SETTINGS.Pos_Y_init").get_parameter_value().double_value
        self.Plane_Pos_z_init = self.get_parameter(f"PLANE_SETTINGS.Pos_Z_init").get_parameter_value().double_value
        self.Plane_Angle_deg_init = self.get_parameter(f"PLANE_SETTINGS.Plane_Angle_init").get_parameter_value().integer_value
        
        # self.get_logger().info(f'Plane_Type: {self.Plane_Type}')
        # self.get_logger().info(f'Plane_Config: {self.Plane_Config}')
        # self.get_logger().info(f'Plane_Pos_x_init: {self.Plane_Pos_x_init}')
        # self.get_logger().info(f'Plane_Pos_y_init: {self.Plane_Pos_y_init}')
        # self.get_logger().info(f'Plane_Pos_z_init: {self.Plane_Pos_z_init}')
        # self.get_logger().info(f'Plane_Angle_deg_init: {self.Plane_Angle_deg_init}')

        ## CONTROLLER GAIN VALUES
        self.P_kp_xy = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.P_kp_xy").get_parameter_value().double_value
        self.P_kd_xy = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.P_kd_xy").get_parameter_value().double_value
        self.P_ki_xy = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.P_ki_xy").get_parameter_value().double_value
        self.i_range_xy = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.i_range_xy").get_parameter_value().double_value

        # self.get_logger().info(f'P_kp_xy: {self.P_kp_xy}')
        # self.get_logger().info(f'P_kd_xy: {self.P_kd_xy}')
        # self.get_logger().info(f'P_ki_xy: {self.P_ki_xy}')
        # self.get_logger().info(f'i_range_xy: {self.i_range_xy}')

        self.P_kp_z = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.P_kp_z").get_parameter_value().double_value
        self.P_kd_z = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.P_kd_z").get_parameter_value().double_value
        self.P_ki_z = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.P_ki_z").get_parameter_value().double_value
        self.i_range_z = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.i_range_z").get_parameter_value().double_value

        # self.get_logger().info(f'P_kp_z: {self.P_kp_z}')
        # self.get_logger().info(f'P_kd_z: {self.P_kd_z}')
        # self.get_logger().info(f'P_ki_z: {self.P_ki_z}')
        # self.get_logger().info(f'i_range_z: {self.i_range_z}')

        self.R_kp_xy = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.R_kp_xy").get_parameter_value().double_value
        self.R_kd_xy = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.R_kd_xy").get_parameter_value().double_value
        self.R_ki_xy = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.R_ki_xy").get_parameter_value().double_value
        self.i_range_R_xy = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.i_range_R_xy").get_parameter_value().double_value

        # self.get_logger().info(f'R_kp_xy: {self.R_kp_xy}')
        # self.get_logger().info(f'R_kd_xy: {self.R_kd_xy}')
        # self.get_logger().info(f'R_ki_xy: {self.R_ki_xy}')
        # self.get_logger().info(f'i_range_R_xy: {self.i_range_R_xy}')

        self.R_kp_z = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.R_kp_z").get_parameter_value().double_value
        self.R_kd_z = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.R_kd_z").get_parameter_value().double_value
        self.R_ki_z = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.R_ki_z").get_parameter_value().double_value
        self.i_range_R_z = self.get_parameter(f"SAR_Type.{self.SAR_Type}.CtrlGains.i_range_R_z").get_parameter_value().double_value

        # self.get_logger().info(f'R_kp_z: {self.R_kp_z}')
        # self.get_logger().info(f'R_kd_z: {self.R_kd_z}')
        # self.get_logger().info(f'R_ki_z: {self.R_ki_z}')
        # self.get_logger().info(f'i_range_R_z: {self.i_range_R_z}')

    def declare_parameters_from_dict(self, parameters, parent_key=''):
        for key, value in parameters.items():
            full_key = f'{parent_key}.{key}' if parent_key else key
            if isinstance(value, dict):
                self.declare_parameters_from_dict(value, full_key)
            else:
                if not self.has_parameter(full_key):
                    self.declare_parameter(full_key, value)

    def _getTime(self):
        """Returns current known time.

        Returns:
            float: Current known time.
        """        
        
        return self.t

    def sendCmd(self,action,cmd_vals=[0.0,0.0,0.0],cmd_flag=1):
        """Sends commands to SAR_DC->Controller via rosservice call

        Args:
            action (string): The desired command
            cmd_vals (list, optional): Command values typically in [x,y,z] notation. Defaults to [0,0,0].
            cmd_flag (float, optional): Used as either a on/off flag for command or an extra float value if needed. Defaults to 1.
        """        
        print("sendCmd in SAR_Base_Interface.py is started")

        ## CREATE SERVICE REQUEST MSG
        srv = CTRLCmdSrv.Request() 

        srv.cmd_type = self.cmd_dict[action]
        srv.cmd_vals.x = cmd_vals[0]
        srv.cmd_vals.y = cmd_vals[1]
        srv.cmd_vals.z = cmd_vals[2]
        srv.cmd_flag = cmd_flag
        srv.cmd_rx = True

        ## TO SAR_DataConverter
        print(f"Sending request: cmd_type={srv.cmd_type}, cmd_vals=({srv.cmd_vals.x}, {srv.cmd_vals.y}, {srv.cmd_vals.z}), cmd_flag={srv.cmd_flag}, cmd_rx={srv.cmd_rx}")
        result = self.callService('/SAR_DC/CMD_Input', srv, CTRLCmdSrv)

        if result:
            print(f"Service call succeeded: srv_success={result.srv_success}")
        else:
            print("Service call failed")
        print("sendCmd in SAR_Base_Interface.py is completed")

        # """Sends commands to SAR_DC->Controller via rosservice call

        # Args:
        #     action (string): The desired command
        #     cmd_vals (list, optional): Command values typically in [x,y,z] notation. Defaults to [0,0,0].
        #     cmd_flag (float, optional): Used as either a on/off flag for command or an extra float value if needed. Defaults to 1.
        # """        
        # print("sendCmd in SAR_Base_Interface.py is started")
        

        # ## CREATE SERVICE REQUEST MSG
        # srv = CTRLCmdSrv.Request() 

        # srv.cmd_type = self.cmd_dict[action]
        # srv.cmd_vals.x = cmd_vals[0]
        # srv.cmd_vals.y = cmd_vals[1]
        # srv.cmd_vals.z = cmd_vals[2]
        # srv.cmd_flag = cmd_flag
        # srv.cmd_rx = True

        # ## TO SAR_DataConverter
        # self.callService('/SAR_DC/CMD_Input',srv,CTRLCmdSrv)    
        # print("sendCmd in SAR_Base_Interface.py is completed")

    ## try several time for service


    def callService(self, srv_addr, srv_msg, srv_type, num_retries=5):
        print("callService in SAR_Base_Interface.py is started")
        cli = self.create_client(srv_type, srv_addr)

        # CHECK THAT SERVICE IS AVAILABLE
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"[WARNING] Service '{srv_addr}' not available, waiting...")

        for retry in range(num_retries):
            try:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                print(f"Attempting to call service '{srv_addr}', attempt {retry + 1}")
                future = cli.call_async(srv_msg)
                loop.run_until_complete(future)

                if future.result() is not None:
                    print(f"Service call to '{srv_addr}' succeeded on attempt {retry + 1}")
                    print(f"Service response: {future.result()}")
                    loop.close()
                    return future.result()
                else:
                    self.get_logger().warn(f"[WARNING] Attempt {retry + 1} to call service '{srv_addr}' failed: {future.exception()}")
                    print(f"[WARNING] Attempt {retry + 1} to call service '{srv_addr}' failed: {future.exception()}")
            except RuntimeError as e:
                self.get_logger().error(f"[ERROR] Runtime error on attempt {retry + 1}: {e}")
                print(f"Runtime error on attempt {retry + 1}: {e}")
            except Exception as e:
                self.get_logger().error(f"[ERROR] Unexpected error on attempt {retry + 1}: {e}")
                print(f"Unexpected error on attempt {retry + 1}: {e}")
            finally:
                loop.close()

        self.Done = True
        self.get_logger().error(f"Service '{srv_addr}' call failed after {num_retries} attempts")
        print(f"Service '{srv_addr}' call failed after {num_retries} attempts")
        
        print("callService in SAR_Base_Interface.py is completed")
        return None

        # print("callService in SAR_Base_Interface.py is started")
        # cli = self.create_client(srv_type, srv_addr)

        # # CHECK THAT SERVICE IS AVAILABLE
        # while not cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info(f"[WARNING] Service '{srv_addr}' not available, waiting...")

        # for retry in range(num_retries):
        #     self.future = cli.call_async(srv_msg)

        #     try:
        #         # Create an event loop if it doesn't exist in the current thread.
        #         loop = asyncio.new_event_loop()
        #         asyncio.set_event_loop(loop)
        #         loop.run_until_complete(self.future)

        #         if self.future.result() is not None:
        #             return self.future.result()
        #         else:
        #             self.get_logger().warn(f"[WARNING] Attempt {retry + 1} to call service '{srv_addr}' failed: {self.future.exception()}")
        #     except RuntimeError as e:
        #         self.get_logger().error(f"[ERROR] Runtime error: {e}")

        # # IF SERVICE CALL FAILS THEN MARK SIM EPISODE AS DONE AND RETURN ERROR
        # self.Done = True
        # self.get_logger().error(f"Service '{srv_addr}' call failed after {num_retries} attempts")
        # print("callService in SAR_Base_Interface.py is completed")
        # return None


    def setAngAcc_range(self, Ang_Acc_range):
        """Sets the range of allowable angular accelerations for the model

        Args:
            Ang_Acc_range (list): List of min/max angular accelerations [min, max]
        """
        if max(abs(i) for i in Ang_Acc_range) > self.Ang_Acc_max:
            self.get_logger().warn(f"Angular Acceleration range exceeds max value of {self.Ang_Acc_max:.0f} deg/s^2")
            self.get_logger().warn("Setting Angular Acceleration range to max value")
            self.Ang_Acc_range = [-self.Ang_Acc_max, self.Ang_Acc_max]
        else:
            self.Ang_Acc_range = Ang_Acc_range

    def startPos_ImpactTraj(self,V_B_P,Acc=None,Tau_CR_start=1.0):

        ## DESIRED RELATIVE VELOCITY VALUES
        V_tx,_,V_perp = V_B_P

        ## CALCULATE STARTING TAU VALUE
        Tau_Body_start = (Tau_CR_start + self.Collision_Radius/V_perp) # Tau read by body
        Tau_Bonus = 0.35 # Bonus time to ensure smooth flight

        ## CALC STARTING POSITION IN GLOBAL COORDS
        # (Derivation: Research_Notes_Book_3.pdf (9/17/23))
        r_P_O = np.array(self.r_P_O)                        # Plane Position wrt to Origin - {X_W,Y_W,Z_W}
        r_P_B = np.array([(Tau_CR_start + Tau_Bonus)*V_tx,
                          0,
                          (Tau_Body_start + Tau_Bonus)*V_perp])             # Body Position wrt to Plane - {t_x,t_y,n_p}
        r_B_O = r_P_O - self.R_PW(r_P_B,self.Plane_Angle_rad)   # Body Position wrt to Origin - {X_W,Y_W,Z_W}



        ## DESIRED GLOBAL VELOCITY VALUES
        V_B_O = self.R_PW(V_B_P,self.Plane_Angle_rad)           # Body Velocity wrt to Origin - {X_W,Y_W,Z_W}


        ## DESIRED ACCELERATION VALUES
        if Acc == None:
            Acc = self.TrajAcc_Max
    
        a_x = Acc[0]
        a_z = Acc[2]


        ## CALC OFFSET POSITIONS
        t_x = V_B_O[0]/a_x    # Time required to reach Vx
        t_z = V_B_O[2]/a_z    # Time required to reach Vz

        x_0 = r_B_O[0] - V_B_O[0]**2/(2*a_x) - V_B_O[0]*t_z     # X-position Vel reached
        y_0 = r_B_O[1]                                          # Y-position Vel reached
        z_0 = r_B_O[2] - V_B_O[2]**2/(2*a_z)                    # Z-position Vel reached    

        return [x_0,y_0,z_0]

    def _setPlanePose(self,Position=[0,0,2.0],Plane_Angle=180):
        print()

    def _sampleFlightConditions(self,V_mag_range=[0.5,1.5],V_angle_range=[0,180]):
        print()

    def userInput(self,input_string,dataType=float):
        """Processes user input and return values as either indiviual value or list

        Args:
            input_string (string): String received from user
            dataType (dataType, optional): Datatype to parse string to. Defaults to float.

        Returns:
            vals: Values parsed by ','. If multiple values then return list
        """        

        while True:
            try:
                vals = [dataType(i) for i in input(input_string).split(',')]
            except:
                continue
        
            ## RETURN MULTIPLE VALUES IF MORE THAN ONE
            if len(vals) == 1:
                return vals[0]
            else:
                return vals  


    # ============================
    ##      Command Handlers 
    # ============================
    def handle_Ctrl_Reset(self):
        print("Reset controller to default values\n")
        #self.sendCmd("GZ_StickyPads",cmd_flag=0)
        self.sendCmd("Ctrl_Reset", cmd_vals=[1.0,1.0,1.0])

    def handle_Pos_Cmd(self):
        cmd_vals = self.userInput("Set desired position values (x,y,z): ",float)
        cmd_flag = self.userInput("Pos control On/Off (1,0): ",int)
        self.sendCmd("Pos",cmd_vals,cmd_flag)

    def handle_Vel_Cmd(self):
        cmd_vals = self.userInput("Set desired velocity values (x,y,z): ",float)
        cmd_flag = self.userInput("Vel control On/Off (1,0): ",int)
        self.sendCmd("Vel",cmd_vals,cmd_flag)

    def handle_Ang_Accel(self):
        cmd_vals = self.userInput("Set desired angular acceleration values (x,y,z): ",float)
        cmd_flag = self.userInput("Ang_Accel control On/Off (1,0): ",int)
        self.sendCmd("Ang_Accel",cmd_vals,cmd_flag)

    def handle_Policy(self):
        action_vals = self.userInput("Set desired policy actions (a_Trg,a_Rot): ",float)
        scale_vals = self.userInput("Set desired a_Rot scaling (a_Rot_low,a_Rot_high): ",float)
        cmd_vals = [action_vals[0],action_vals[1],scale_vals[0]]
        cmd_flag = scale_vals[1]
        self.sendCmd("Policy",cmd_vals,cmd_flag)

    def handle_Plane_Pose(self):
        cmd_vals = self.userInput("Set desired position values (x,y,z): ",float)
        cmd_flag = self.userInput("Set desired plane angle [deg]: ",float)
        self._setPlanePose(cmd_vals,cmd_flag)

    ## ========== TRAJECTORY FUNCTIONS ==========
        
    def handle_P2P_traj(self):
        x_d = self.userInput("Desired position (x,y,z):",float)
        self.sendCmd('P2P_traj',cmd_vals=[np.nan,x_d[0],0.5],cmd_flag=0)
        self.sendCmd('P2P_traj',cmd_vals=[np.nan,x_d[1],0.5],cmd_flag=1)
        self.sendCmd('P2P_traj',cmd_vals=[np.nan,x_d[2],0.5],cmd_flag=2)
        self.sendCmd('Activate_traj',cmd_vals=[1.0,1.0,1.0])

    def handle_Global_Vel_traj(self):

        ## GET GLOBAL VEL CONDITIONS 
        V_mag,V_angle = self.userInput("Flight Velocity (V_mag,V_angle):",float)

        ## CALC GLOBAL VELOCITIES
        Vx = V_mag*np.cos(np.radians(V_angle))
        Vy = 0
        Vz = V_mag*np.sin(np.radians(V_angle))
        V_B_O = [Vx,Vy,Vz]

        ## EXECUTE TRAJECTORY
        self.sendCmd('Const_Vel_traj',cmd_vals=[V_B_O[0],self.TrajAcc_Max[0],self.TrajJerk_Max[0]],cmd_flag=0)
        self.sendCmd('Const_Vel_traj',cmd_vals=[V_B_O[2],self.TrajAcc_Max[2],self.TrajJerk_Max[2]],cmd_flag=2)
        self.sendCmd('Activate_traj',cmd_vals=[1.0,0.0,1.0])

    def handle_Rel_Vel_traj(self):

        ## GET RELATIVE VEL CONDITIONS 
        V_mag,V_angle = self.userInput("Flight Velocity (V_mag,V_angle):",float)

        ## CALC RELATIVE VELOCITIES
        V_tx = V_mag*np.cos(np.radians(V_angle))
        V_ty = 0
        V_perp = V_mag*np.sin(np.radians(V_angle))

        ## CALCULATE GLOBAL VELOCITIES
        V_B_O = self.R_PW(np.array([V_tx,V_ty,V_perp]),self.Plane_Angle_rad)

        ## EXECUTE TRAJECTORY
        self.sendCmd('Const_Vel_traj',cmd_vals=[V_B_O[0],self.TrajAcc_Max[0],self.TrajJerk_Max[0]],cmd_flag=0)
        self.sendCmd('Const_Vel_traj',cmd_vals=[V_B_O[2],self.TrajAcc_Max[2],self.TrajJerk_Max[2]],cmd_flag=2)
        self.sendCmd('Activate_traj',cmd_vals=[1,0,1])

    def handle_Impact_traj(self):

        ## GET RELATIVE VEL CONDITIONS 
        V_mag,V_angle = self.userInput("Flight Velocity (V_mag,V_angle):",float)

        ## CALC RELATIVE VELOCITIES
        V_x = V_mag*np.cos(np.radians(V_angle))
        #print("V_tx value : ", V_tx)
        V_y = 0
        V_z = V_mag*np.sin(np.radians(V_angle))
        #print("V_perp value : ", V_perp)
        V_B_O = np.array([V_x,V_y,V_z])
        #print("V_B_P value : ", V_B_P)

        ## CALCULATE GLOBAL VELOCITIES
        V_B_P = self.R_WP(V_B_O,self.Plane_Angle_rad)
        #print("Plane_Angle_rad value : ", self.Plane_Angle_rad)
        

        ## POS VELOCITY CONDITIONS MET
        r_B_O = self.startPos_ImpactTraj(V_B_P,Acc=None)

        ## APPROVE START POSITION
        print(YELLOW,f"Start Position: ({r_B_O[0]:.2f},{self.r_B_O[1]:.2f},{r_B_O[2]:.2f})",RESET)
        str_input = self.userInput("Approve start position (y/n): ",str)
        if str_input == 'y':
            self.sendCmd('P2P_traj',cmd_vals=[np.nan,r_B_O[0],0.5],cmd_flag=0)
            self.sendCmd('P2P_traj',cmd_vals=[np.nan,r_B_O[1],0.5],cmd_flag=1)
            self.sendCmd('P2P_traj',cmd_vals=[np.nan,r_B_O[2],0.5],cmd_flag=2)
            self.sendCmd('Activate_traj',cmd_vals=[1.0,1.0,1.0])
        else:
            raise Exception("Start position not approved")

        ## POLICY SENDING
        cmd_vals = self.userInput("Set desired (Tau,AngAcc) Policy: ",float)
        cmd_vals.append(-100.0) # Append extra value to match framework
        self.sendCmd('Policy',cmd_vals,cmd_flag=0)

        ## APPROVE FLIGHT
        str_input = self.userInput("Approve flight (y/n): ",str)
        if str_input == 'y':
            self.sendCmd('Const_Vel_traj',cmd_vals=[V_B_O[0],self.TrajAcc_Max[0],self.TrajJerk_Max[0]],cmd_flag=0)
            self.sendCmd('Const_Vel_traj',cmd_vals=[V_B_O[2],self.TrajAcc_Max[2],self.TrajJerk_Max[2]],cmd_flag=2)
            self.sendCmd('Activate_traj',cmd_vals=[1.0,0.0,1.0])
        else:
            raise Exception("Flight not approved")
            

    ## ========== SYSTEM FUNCTIONS ========== 
            
    def handle_Arm_Quad(self):

        #cmd_flag = self.userInput("Arm Quad On/Off (1,0): ",int)
        cmd_flag = self.userInput("Arm Quad On/Off (1,0): ",int)
        
        #For experiment
        #self.sendCmd("Load_Params")
        #self.sendCmd("Ctrl_Reset")
        #self.sendCmd("Plane_Pose",cmd_vals=[self.Plane_Pos_x_init,self.Plane_Pos_y_init,self.Plane_Pos_z_init],cmd_flag=self.Plane_Angle_deg_init)
        
        self.sendCmd("Arm_Quad",cmd_vals=[1.0,1.0,1.0], cmd_flag=cmd_flag)
        
    
    def handle_Tumble_Detect(self):

        cmd_flag = self.userInput("Tumble Detection On/Off (1,0): ",int)
        self.sendCmd('Tumble_Detect',cmd_vals=[1.0,1.0,1.0],cmd_flag=cmd_flag)

    def handle_Load_Params(self):

        print("Reset ROS Parameters\n")
        self.loadBaseParams()
        self.sendCmd("Load_Params")

    def handle_Start_Logging(self):

        self.startLogging()

    def handle_Cap_Logging(self):

        self.capLogging()

    
    ## ========== MOTOR FUNCTIONS ==========
        
    def handle_Stop(self):
        #!!!!! minor changed with Bryan's program added cmd_vals=[1.0,1.0,1.0]
        self.sendCmd("Stop", cmd_vals=[1.0,1.0,1.0])

    def handle_Thrust_CMD(self):
        vals = self.userInput("Set desired thrust values (f1,f2,f3,f4): ",float)
        self.sendCmd("Thrust_Cmd",vals[:3],vals[3])
        
    def handle_Motor_CMD(self):
        vals = self.userInput("Set desired motor values (m1,m2,m3,m4): ",float)
        self.sendCmd("Motor_Cmd",vals[:3],vals[3])

    # ========================
    ##    Logging Services 
    # ========================

    def createCSV(self,logName):
        """Sends service to CF_DataConverter to create CSV log file 

        Args:
            filePath (string): Send full path and file name to write to
        """      

        ## CREATE SERVICE REQUEST MSG
        srv = LoggingCMD.Request()
        srv.file_path = os.path.join(self.Log_Dir,logName)
        srv.logging_cmd = 0

        ## SEND LOGGING REQUEST VIA SERVICE
        self.callService('/SAR_DC/DataLogging',srv,LoggingCMD)

    def startLogging(self,logName):
        """Start logging values to the current CSV file
        """        

        ## CREATE SERVICE REQUEST MSG
        srv = LoggingCMD.Request()
        srv.file_path = os.path.join(self.Log_Dir,logName)
        srv.logging_cmd = 1

        ## SEND LOGGING REQUEST VIA SERVICE
        self.callService('/SAR_DC/DataLogging',srv,LoggingCMD)

    def capLogging(self,logName):
        """Cap logging values with Flight, Rot, and Impact conditions and stop continuous logging
        """        

        ## CREATE SERVICE REQUEST MSG
        srv = LoggingCMD.Request()
        srv.file_path = os.path.join(self.Log_Dir,logName)
        srv.logging_cmd = 2
        srv.error_string = self.Error_Str # String for why logging was capped
        
        ## SEND LOGGING REQUEST VIA SERVICE
        self.callService('/SAR_DC/DataLogging',srv,LoggingCMD)

    # ============================
    ##   Publishers/Subscribers 
    # ============================
    def _preInit(self):

        ## COMMAND DICT
        self.cmd_dict = {
            'Ctrl_Reset':0,
            'Pos':1,
            'Vel':2,

            'Stop':5,
            'Ang_Accel':7,
            'Policy':8,
            'Plane_Pose':9,

            'P2P_traj':10,
            'Const_Vel_traj':11,
            'Impact_traj':13,
            'Activate_traj':19,

            'Tumble_Detect':20,
            'Load_Params':21,
            'Start_Logging':22,
            'Cap_Logging':23,
            'Arm_Quad':24,

            'Thrust_CMD':30,
            'Motor_CMD':31,

            'GZ_Pose_Reset':90,
            'GZ_StickyPads':91,
            'GZ_Const_Vel_Traj':92,
        }
        self.inv_cmd_dict = {value: key for key, value in self.cmd_dict.items()}

        ## RL BASED VALUES
        self.K_ep = np.nan
        self.K_run = np.nan
        self.Error_Str = "No_Debug_Data"
        self.n_rollouts = np.nan

        self.policy = np.full([3],np.nan)
        self.reward = np.nan
        self.reward_avg = np.nan
        self.reward_vals = np.full([6],np.nan)

        self.K_ep_list = []
        self.K_run_list = []

        self.mu_1_list = []
        self.mu_2_list = []

        self.sigma_1_list = []
        self.sigma_2_list = []

        self.reward_list = []
        self.Kep_list_reward_avg = []
        self.reward_avg_list = []

        ## STATE DATA VALUES
        self.t = np.nan
        self.r_B_O = np.full([3],np.nan)
        self.V_B_O = np.full([3],np.nan)
        self.eul_B_O = np.full([3],np.nan)
        self.omega_B_O = np.full([3],np.nan)

        self.r_P_B = np.full([3],np.nan)
        self.Eul_P_B = np.full([3],np.nan)
        self.V_B_P = np.full([3],np.nan)
        self.Omega_B_P = np.full([3],np.nan)
        self.Vel_mag_B_P = np.nan
        self.Vel_angle_B_P = np.nan
        self.D_perp = np.nan
        self.D_perp_CR = np.nan
        self.D_perp_pad = np.nan
        self.D_perp_pad_min = np.nan
        
        self.Theta_x = np.nan
        self.Theta_y = np.nan
        self.Tau = np.nan
        self.Tau_CR = np.nan

        ## TRIGGER DATA VALUES
        self.Trg_Flag = False
        self.t_trg = np.nan
        self.r_B_O_trg = np.full([3],np.nan)
        self.V_B_O_trg = np.full([3],np.nan)
        self.Eul_B_O_trg = np.full([3],np.nan)
        self.Omega_B_O_trg = np.full([3],np.nan)

        self.r_P_B_trg = np.full([3],np.nan)
        self.Eul_P_B_trg = np.full([3],np.nan)
        self.V_B_P_trg = np.full([3],np.nan)
        self.Omega_B_P_trg = np.full([3],np.nan)
        self.D_perp_trg = np.nan
        self.D_perp_CR_trg = np.nan

        self.Theta_x_trg = np.nan
        self.Theta_y_trg = np.nan
        self.Tau_trg = np.nan
        self.Tau_CR_trg = np.nan

        self.Vel_mag_B_O_trg = np.nan
        self.Vel_angle_B_O_trg = np.nan

        self.Vel_mag_B_P_trg = np.nan
        self.Vel_angle_B_P_trg = np.nan

        self.NN_Output_trg = np.full([4],np.nan)
        self.a_Trg_trg = np.nan
        self.a_Rot_trg = np.nan

        ## IMPACT DATA VALUES
        self.Impact_Flag = False
        
        self.Impact_Flag_OB = False
        self.t_impact_OB = np.nan
        self.r_B_O_impact_OB = np.full([3],np.nan)
        self.Eul_B_O_impact_OB = np.full([3],np.nan)
        self.V_B_P_impact_OB = np.full([3],np.nan)
        self.Omega_B_P_impact_OB = np.full([3],np.nan)
        self.Eul_P_B_impact_OB = np.full([3],np.nan)

        self.Impact_Flag_Ext = False
        self.BodyContact_Flag = False
        self.ForelegContact_Flag = False
        self.HindlegContact_Flag = False
        self.t_impact_Ext = np.nan
        self.r_B_O_impact_Ext = np.full([3],np.nan)
        self.Eul_B_O_impact_Ext = np.full([3],np.nan)
        self.V_B_P_impact_Ext = np.full([3],np.nan)
        self.Omega_B_P_impact_Ext = np.full([3],np.nan)
        self.Eul_P_B_impact_Ext = np.full([3],np.nan)
        self.Rot_Sum_impact_Ext = np.nan

        self.Force_impact_x = np.nan
        self.Force_impact_y = np.nan
        self.Force_impact_z = np.nan
        self.Impact_Magnitude = np.nan

        self.Pad_Connections = np.nan
        self.Pad1_Contact = np.nan
        self.Pad2_Contact = np.nan
        self.Pad3_Contact = np.nan
        self.Pad4_Contact = np.nan

        ## MISC DATA VALUES
        self.V_Battery = np.nan

        self.Plane_Angle_deg = np.nan
        self.Plane_Angle_rad = np.nan
        self.r_P_O = np.full([3],np.nan)

    def _clockCallback(self,msg):
        print()


#!!!! time need to solve
    def _SAR_StateDataCallback(self,StateData_msg):
        #self.get_logger().info(f"Received StateData_msg at time: {StateData_msg.time.sec}.{StateData_msg.time.nanosec}")

        #data_type = self.get_parameter('DATA_TYPE').get_parameter_value().string_value
        
        #if data_type == "EXP":
        #    self.t = StateData_msg.time.sec + StateData_msg.time.nanosec * 1e-9
        ##if rospy.get_param('/DATA_TYPE') == "EXP": ##rospy is for ROS1, DH need to migrate this line
        ##    self.t = StateData_msg.Time.data.to_sec()

        ## STATES WRT ORIGIN
        self.r_B_O = np.round([StateData_msg.pose_b_o.position.x,
                                StateData_msg.pose_b_o.position.y,
                                StateData_msg.pose_b_o.position.z],3)
        #print(f"self.r_B_O: {self.r_B_O[0]:.0f}")
        self.V_B_O = np.round([StateData_msg.twist_b_o.linear.x,
                                StateData_msg.twist_b_o.linear.y,
                                StateData_msg.twist_b_o.linear.z],3)
        
        self.eul_B_O = np.round([StateData_msg.eul_b_o.x,
                                    StateData_msg.eul_b_o.y,
                                    StateData_msg.eul_b_o.z],3)
        self.omega_B_O = np.round([StateData_msg.twist_b_o.angular.x,
                                    StateData_msg.twist_b_o.angular.y,
                                    StateData_msg.twist_b_o.angular.z],3)
        
        ## STATES WRT PLANE
        self.r_P_B = np.round([StateData_msg.pose_p_b.position.x,
                                StateData_msg.pose_p_b.position.y,
                                StateData_msg.pose_p_b.position.z],3)
        
        self.Eul_P_B = np.round([StateData_msg.eul_p_b.x,
                                    StateData_msg.eul_p_b.y,
                                    StateData_msg.eul_p_b.z],3)
        
        self.V_B_P = np.round([StateData_msg.twist_b_p.linear.x,
                                StateData_msg.twist_b_p.linear.y,
                                StateData_msg.twist_b_p.linear.z],3)
        
        self.Omega_B_P = np.round([StateData_msg.twist_b_p.angular.x,
                                    StateData_msg.twist_b_p.angular.y,
                                    StateData_msg.twist_b_p.angular.z],3)
        
        self.Vel_mag_B_P = np.round(StateData_msg.vel_mag_b_p,3)
        self.Vel_angle_B_P = np.round(StateData_msg.vel_angle_b_p,3)
        self.D_perp = np.round(StateData_msg.d_perp,3)
        self.D_perp_CR = np.round(StateData_msg.d_perp_cr,3)
        self.D_perp_pad = np.round(StateData_msg.d_perp_pad,3)
        self.D_perp_pad_min = np.round(StateData_msg.d_perp_pad_min,3)

        ## OPTICAL FLOW STATES
        self.Theta_x = np.round(StateData_msg.optical_flow.x,3)
        self.Theta_y = np.round(StateData_msg.optical_flow.y,3)
        self.Tau = np.round(StateData_msg.optical_flow.z,3)
        self.Tau_CR = np.round(StateData_msg.tau_cr,3)
        
        #self.t_prev = self.t # Save t value for next callback iteration

        #self.get_logger().info("Callback function executed successfully.")


#!!!!! Time and NNoutput need to solve
    def _SAR_TriggerDataCallback(self,TriggerData_msg):

        ## TRIGGER FLAG
        self.Trg_Flag = TriggerData_msg.trg_flag

        ## POLICY TRIGGERING CONDITIONS
        #self.t_trg = TriggerData_msg.Time_trg.data.to_sec()

        ## STATES WRT ORIGIN
        self.r_B_O_trg = np.round([TriggerData_msg.pose_b_o_trg.position.x,
                                    TriggerData_msg.pose_b_o_trg.position.y,
                                    TriggerData_msg.pose_b_o_trg.position.z],3)
        
        self.V_B_O_trg = np.round([TriggerData_msg.twist_b_o_trg.linear.x,
                                    TriggerData_msg.twist_b_o_trg.linear.y,
                                    TriggerData_msg.twist_b_o_trg.linear.z],3)
        
        self.Eul_B_O_trg = np.round([TriggerData_msg.eul_b_o_trg.x,
                                    TriggerData_msg.eul_b_o_trg.y,
                                    TriggerData_msg.eul_b_o_trg.z],3)
        
        self.Omega_B_O_trg = np.round([TriggerData_msg.twist_b_o_trg.angular.x,
                                        TriggerData_msg.twist_b_o_trg.angular.y,
                                        TriggerData_msg.twist_b_o_trg.angular.z],3)

        ## STATES WRT PLANE
        self.r_P_B_trg = np.round([TriggerData_msg.pose_p_b_trg.position.x,
                                    TriggerData_msg.pose_p_b_trg.position.y,
                                    TriggerData_msg.pose_p_b_trg.position.z],3)
        
        self.Eul_P_B_trg = np.round([TriggerData_msg.eul_p_b_trg.x,
                                    TriggerData_msg.eul_p_b_trg.y,
                                    TriggerData_msg.eul_p_b_trg.z],3)
        
        self.V_B_P_trg = np.round([TriggerData_msg.twist_b_p_trg.linear.x,  
                                    TriggerData_msg.twist_b_p_trg.linear.y,
                                    TriggerData_msg.twist_b_p_trg.linear.z],3)
        
        self.Omega_B_P_trg = np.round([TriggerData_msg.twist_b_p_trg.angular.x,
                                        TriggerData_msg.twist_b_p_trg.angular.y,
                                        TriggerData_msg.twist_b_p_trg.angular.z],3)
        
        self.D_perp_trg = np.round(TriggerData_msg.d_perp_trg,3)
        self.D_perp_CR_trg = np.round(TriggerData_msg.d_perp_cr_trg,3)

        ## OPTICAL FLOW STATES
        self.Theta_x_trg = np.round(TriggerData_msg.optical_flow_trg.x,3)
        self.Theta_y_trg = np.round(TriggerData_msg.optical_flow_trg.y,3)
        self.Tau_trg = np.round(TriggerData_msg.optical_flow_trg.z,3)
        self.Tau_CR_trg = np.round(TriggerData_msg.tau_cr_trg,3)

        self.Vel_mag_B_O_trg = np.round(TriggerData_msg.vel_mag_b_o_trg,3)
        self.Vel_angle_B_O_trg = np.round(TriggerData_msg.vel_angle_b_o_trg,3)

        self.Vel_mag_B_P_trg = np.round(TriggerData_msg.vel_mag_b_p_trg,3)
        self.Vel_angle_B_P_trg = np.round(TriggerData_msg.vel_angle_b_p_trg,3)

        ## POLICY TRIGGERING CONDITIONS
        #self.NN_Output_trg = np.round(TriggerData_msg.NN_Output_trg,3)
        self.a_Trg_trg = np.round(TriggerData_msg.a_trg_trg,3)
        self.a_Rot_trg = np.round(TriggerData_msg.a_rot_trg,3)

    def _SAR_ImpactDataCallback(self,ImpactData_msg):

        self.Impact_Flag = ImpactData_msg.impact_flag


        ## ONBOARD IMPACT DETECTION
        self.Impact_Flag_OB = ImpactData_msg.impact_flag_ob
        #self.t_impact_OB = ImpactData_msg.Time_impact_OB.data.to_sec()

        self.r_B_O_impact_OB = np.round([ImpactData_msg.pose_b_o_impact_ob.position.x,
                                        ImpactData_msg.pose_b_o_impact_ob.position.y,
                                        ImpactData_msg.pose_b_o_impact_ob.position.z],3)
        
        self.Eul_B_O_impact_OB = np.round([ImpactData_msg.eul_b_o_impact_ob.x,
                                            ImpactData_msg.eul_b_o_impact_ob.y,
                                            ImpactData_msg.eul_b_o_impact_ob.z],3)
        
        self.V_B_P_impact_OB = np.round([ImpactData_msg.twist_b_p_impact_ob.linear.x,
                                        ImpactData_msg.twist_b_p_impact_ob.linear.y,
                                        ImpactData_msg.twist_b_p_impact_ob.linear.z],3)
        
        self.Omega_B_P_impact_OB = np.round([ImpactData_msg.twist_b_p_impact_ob.angular.x,
                                            ImpactData_msg.twist_b_p_impact_ob.angular.y,
                                            ImpactData_msg.twist_b_p_impact_ob.angular.z],3)
        
        self.Eul_P_B_impact_OB = np.round([ImpactData_msg.eul_p_b_impact_ob.x,
                                        ImpactData_msg.eul_p_b_impact_ob.y,
                                        ImpactData_msg.eul_p_b_impact_ob.z],3)
        
        

        ## EXTERNAL IMPACT DETECTION
        self.Impact_Flag_Ext = ImpactData_msg.impact_flag_ext
        self.BodyContact_Flag = ImpactData_msg.bodycontact_flag
        self.ForelegContact_Flag = ImpactData_msg.forelegcontact_flag
        self.HindlegContact_Flag = ImpactData_msg.hindlegcontact_flag
        
        #self.t_impact_Ext = ImpactData_msg.Time_impact_Ext.data.to_sec()

        self.r_B_O_impact_Ext = np.round([ImpactData_msg.pose_b_o_impact_ext.position.x,
                                        ImpactData_msg.pose_b_o_impact_ext.position.y,
                                        ImpactData_msg.pose_b_o_impact_ext.position.z],3)
        
        self.Eul_B_O_impact_Ext = np.round([ImpactData_msg.eul_b_o_impact_ext.x,
                                            ImpactData_msg.eul_b_o_impact_ext.y,
                                            ImpactData_msg.eul_b_o_impact_ext.z],3)
        
        self.V_B_P_impact_Ext = np.round([ImpactData_msg.twist_b_p_impact_ext.linear.x,
                                        ImpactData_msg.twist_b_p_impact_ext.linear.y,
                                        ImpactData_msg.twist_b_p_impact_ext.linear.z],3)
        
        self.Omega_B_P_impact_Ext = np.round([ImpactData_msg.twist_b_p_impact_ext.angular.x,
                                            ImpactData_msg.twist_b_p_impact_ext.angular.y,
                                            ImpactData_msg.twist_b_p_impact_ext.angular.z],3)
        
        self.Eul_P_B_impact_Ext = np.round([ImpactData_msg.eul_p_b_impact_ext.x,
                                        ImpactData_msg.eul_p_b_impact_ext.y,
                                        ImpactData_msg.eul_p_b_impact_ext.z],3)
        
        self.Rot_Sum_impact_Ext = np.round(ImpactData_msg.rot_sum_impact_ext,3)

        
        ## IMPACT FORCES
        self.Force_impact_x = np.round(ImpactData_msg.force_impact.x,3)
        self.Force_impact_y = np.round(ImpactData_msg.force_impact.y,3)
        self.Force_impact_z = np.round(ImpactData_msg.force_impact.z,3)
        self.Impact_Magnitude = np.round(ImpactData_msg.impact_magnitude,3)

        ## PAD CONNECTIONS
        self.Pad_Connections = ImpactData_msg.pad_connections
        self.Pad1_Contact = ImpactData_msg.pad1_contact
        self.Pad2_Contact = ImpactData_msg.pad2_contact
        self.Pad3_Contact = ImpactData_msg.pad3_contact
        self.Pad4_Contact = ImpactData_msg.pad4_contact

    def _SAR_MiscDataCallback(self,MiscData_msg):    

        self.V_Battery = np.round(MiscData_msg.battery_voltage,4)

        self.Plane_Angle_deg = MiscData_msg.plane_angle
        self.Plane_Angle_rad = np.deg2rad(self.Plane_Angle_deg)
        self.r_P_O = [
            MiscData_msg.plane_pos.x,
            MiscData_msg.plane_pos.y,
            MiscData_msg.plane_pos.z,
        ]

    def _RL_Publish(self):
        print()

    def _ROS_PARAMETERCallback(self,ROSParams_msg):
        
        ## SAR PARAMETERS
        self.SAR_Type = ROSParams_msg.data_type
        self.SAR_Config = ROSParams_msg.sar_config
        self.Policy_Type = ROSParams_msg.policy_type
        
        #self.Pos_0 = [0.0, 0.0, 0.4] 
        
        ## INERTIAL PARAMETERS
        self.Ref_Mass = ROSParams_msg.ref_mass
        self.Ref_Ixx = ROSParams_msg.ref_ixx
        self.Ref_Iyy = ROSParams_msg.ref_iyy
        self.Ref_Izz = ROSParams_msg.ref_izz

        self.Base_Mass = ROSParams_msg.base_mass
        self.Base_Ixx = ROSParams_msg.base_ixx
        self.Base_Iyy = ROSParams_msg.base_iyy
        self.Base_Izz = ROSParams_msg.base_izz

        ## GEOMETRIC PARAMETERS
        self.Forward_Reach = ROSParams_msg.forward_reach
        self.Leg_Length = ROSParams_msg.leg_length
        self.Leg_Angle = ROSParams_msg.leg_angle
        self.Prop_Front = ROSParams_msg.prop_front_vec
        self.Prop_Rear = ROSParams_msg.prop_rear_vec

        ## EFFECTIVE-GEOEMTRIC PARAMETERS
        self.L_eff = ROSParams_msg.l_eff
        self.Gamma_eff = ROSParams_msg.gamma_eff
        self.Lx_eff = self.L_eff*np.sin(np.radians(self.Gamma_eff))
        self.Lz_eff = self.L_eff*np.cos(np.radians(self.Gamma_eff))
        self.Collision_Radius = max(self.L_eff,self.Forward_Reach)

                ## SYSTEM AND FLIGHT PARAMETERS
        self.Thrust_max = ROSParams_msg.thrust_max
        self.TrajAcc_Max = ROSParams_msg.trajacc_max
        self.TrajJerk_Max = ROSParams_msg.trajjerck_max
        self.Tau_up = ROSParams_msg.tau_up
        self.Tau_down = ROSParams_msg.tau_down
        self.Thrust_max = ROSParams_msg.thrust_max
        self.Ang_Acc_max = (9.81*self.Thrust_max*1e-3*self.Prop_Front[0])*2/self.Ref_Iyy
        self.setAngAcc_range([-self.Ang_Acc_max, self.Ang_Acc_max])

        self.Beta_Min_deg = -(self.Gamma_eff + np.degrees(np.arctan2(self.Forward_Reach-self.Lx_eff,self.Lz_eff)))
        self.Phi_P_B_impact_Min_deg = -self.Beta_Min_deg - self.Gamma_eff + 90

        ## CAM PARAMETERS
        self.Cam_Config = ROSParams_msg.cam_config
        self.Cam_Active = ROSParams_msg.camactive_flag

        ## PLANE PARAMETERS
        self.Plane_Type = ROSParams_msg.policy_type
        self.Plane_Config = ROSParams_msg.plane_config
        self.Plane_Pos_x_init = ROSParams_msg.pos_x
        self.Plane_Pos_y_init = ROSParams_msg.pos_y
        self.Plane_Pos_z_init = ROSParams_msg.pos_y
        self.Plane_Angle_deg_init = ROSParams_msg.plane_angle_deg

    # ============================
    ##   Rotation Matrices 
    # ============================
    def R_BW(self,vec,phi):

        R_BW = np.array([
            [ np.cos(phi), 0,     np.sin(phi)],
            [     0,       1,         0      ],
            [-np.sin(phi), 0,     np.cos(phi)],
        ])

        return R_BW.dot(vec)
    
    def R_WB(self,vec,phi):

        R_WB = np.array([
            [ np.cos(phi), 0,    -np.sin(phi)],
            [     0,       1,         0      ],
            [ np.sin(phi), 0,     np.cos(phi)],
        ])

        return R_WB.dot(vec)
    
    def R_WP(self,vec,theta):

        R_WP = np.array([
            [ np.cos(theta), 0, -np.sin(theta)],
            [     0,         1,        0      ],
            [ np.sin(theta), 0,  np.cos(theta)]
        ])

        return R_WP.dot(vec)
    
    def R_PW(self,vec,theta):

        R_PW = np.array([
            [ np.cos(theta), 0, np.sin(theta)],
            [     0,         1,       0      ],
            [-np.sin(theta), 0, np.cos(theta)]
        ])

        return R_PW.dot(vec)
    
    def R_PC1(self,vec,Beta1):

        R_PC1 = np.array([
            [ np.cos(Beta1), 0, -np.sin(Beta1)],
            [     0,         1,       0       ],
            [ np.sin(Beta1), 0,  np.cos(Beta1)]
        ])

        return R_PC1.dot(vec)
    
    def R_C1P(self,vec,Beta1):

        R_C1P = np.array([
            [ np.cos(Beta1), 0, np.sin(Beta1)],
            [     0,         1,       0      ],
            [-np.sin(Beta1), 0, np.cos(Beta1)]
        ])

        return R_C1P.dot(vec)
    
    def R_C1B(self,vec,gamma_rad):

        R_C1B = np.array([
            [ np.sin(gamma_rad), 0, np.cos(gamma_rad)],
            [     0,             1,         0        ],
            [-np.cos(gamma_rad), 0, np.sin(gamma_rad)],
        ])

        return R_C1B.dot(vec)

    def R_PC2(self,vec,Beta2):

        R_PC2 = np.array([
            [ np.cos(Beta2), 0, np.sin(Beta2)],
            [     0,         1,       0      ],
            [-np.sin(Beta2), 0, np.cos(Beta2)]
        ])

        return R_PC2.dot(vec)
    
    def R_C2P(self,vec,Beta2):

        R_C2P = np.array([
            [ np.cos(Beta2), 0, np.sin(Beta2)],
            [     0,         1,       0      ],
            [-np.sin(Beta2), 0, np.cos(Beta2)],
        ])

        return R_C2P.dot(vec)

    def R_C2B(self,vec,gamma_rad):

        R_C2B = np.array([
            [-np.sin(gamma_rad), 0,  np.cos(gamma_rad)],
            [        0,          1,          0        ],
            [-np.cos(gamma_rad), 0, -np.sin(gamma_rad)],
        ])

        return R_C2B.dot(vec)

    def arctan4(self,y, x, rotation_direction=-1):
        angle_radians = np.arctan2(y, x)

        if rotation_direction == -1:
            if angle_radians < 0:
                angle_radians += 2*np.pi
        elif rotation_direction == +1:
            if angle_radians > 0:
                angle_radians = 2*np.pi - angle_radians
            else:
                angle_radians = -angle_radians

        return angle_radians    


def main(args=None):
    rclpy.init(args=args)
    env = SAR_Base_Interface()
    rclpy.spin(env)
    env.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()