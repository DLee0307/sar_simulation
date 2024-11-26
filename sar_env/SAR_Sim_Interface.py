#!/usr/bin/env python3
import numpy as np

import os
import time
import subprocess
from threading import Thread,Event
import rclpy
from rclpy.node import Node
from rclpy.task import Future
import sys

from sar_msgs.srv import CTRLGetObs
from sar_msgs.srv import CTRLGetTcon


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


        ## START SIMULATION
        self._kill_Sim()
        self._restart_Sim()
        self._start_monitoring_subprocesses()
        # self.Sim_Status = "Running"
        self._wait_for_sim_running()
        
        #self._getTick()
        #print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")


    def loadSimParams(self):
        print()

    # =========================
    ##     ENV INTERACTION
    # =========================
    

    def _iterStep(self,n_steps:int = 10):
        """Update simulation by n timesteps

        Args:
            n_steps (int, optional): Number of timesteps to step through. Defaults to 10.
        """

        #self.callService('/ENV/World_Step',World_StepRequest(n_steps),World_Step)
        #gz service -s /world/empty/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: true, multi_step: 200'

        # Pause the simulation
        self.pausePhysics(True)
        time.sleep(0.5)

        # Use subprocess to call the GZ service for pausing and advancing steps
        cmd = [
            'gz', 'service', '-s', '/world/empty/control',
            '--reqtype', 'gz.msgs.WorldControl',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000',
            '--req', f'pause: true, multi_step: {n_steps}'
        ]
        
        # Run the command
        result = subprocess.run(cmd, capture_output=True, text=True)
        
        # # Check the output
        # if result.returncode == 0:
        #     print("Successfully advanced the simulation by", n_steps, "steps.")
        # else:
        #     print("Error:", result.stderr)


    def _getTick(self):
        srv = CTRLGetObs.Request() 
        #print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        result = self.callService('/CTRL/Get_Obs',srv, CTRLGetObs)
        #print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")

        # if result:
        #     print(f"Service call succeeded: srv_success={result.tick}")
        # else:
        #     print("Service call failed")

        return result.tick

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

    def _getObs_DH(self):        
        resp = self.callService('/CTRL/Get_Obs',CTRLGetObs.Request(),CTRLGetObs)

        Tau_CR = resp.tau_cr
        Theta_x = resp.theta_x
        D_perp_CR = resp.d_perp_cr

        obs_list = [Tau_CR,Theta_x,D_perp_CR]

        Tau_CR_scaled = self.scaleValue(Tau_CR,original_range=[-5,5],target_range=[-1,1])
        Theta_x_scaled = self.scaleValue(Theta_x,original_range=[-20,20],target_range=[-1,1])
        D_perp_CR_scaled = self.scaleValue(D_perp_CR,original_range=[-0.5,2.0],target_range=[-1,1])

        scaled_obs_list = [Tau_CR_scaled,Theta_x_scaled,D_perp_CR_scaled]

        ## OBSERVATION VECTOR
        obs = np.array(scaled_obs_list,dtype=np.float32)

        return obs

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
    def Sim_VelTraj(self,pos,vel,t_total:float = 1.0):
        """
        Args:
            pos (list): Launch position [m]   | [x,y,z]
            vel (list): Launch velocity [m/s] | [Vx,Vy,Vz]
        """

        model_name = self.SAR_Config
        x, y, z = pos[0], pos[1], pos[2]

        self.pausePhysics(pause_flag=True)

        # Use subprocess to call the GZ service for setting pose
        # Need to chane "SAR_Config"
        cmd = [
            'gz', 'service', '-s', '/world/empty/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000',
            '--req', f'name: "A30_L200, position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: 0, y: 0, z: 0, w: 1}}'
        ]

        result = subprocess.run(cmd, capture_output=True, text=True)

        # print the result of execution
        # if result.returncode == 0:
        #     print("Pose set successfully:", result.stdout)
        # else:
        #     print("Error setting pose:", result.stderr)

        # Run the command
        self.sendCmd('Pos',cmd_vals=pos,cmd_flag=1.0)
        self._iterStep(100)

        model_name = self.SAR_Config
        x, y, z = pos[0], pos[1], pos[2]

        self.pausePhysics(pause_flag=True)

        # Use subprocess to call the GZ service for setting pose
        # Need to chane "SAR_Config"
        cmd = [
            'gz', 'service', '-s', '/world/empty/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000',
            '--req', f'name: "A30_L200", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: 0, y: 0, z: 0, w: 1}}'
        ]

        result = subprocess.run(cmd, capture_output=True, text=True)

        # print the result of execution
        # if result.returncode == 0:
        #     print("Pose set successfully:", result.stdout)
        # else:
        #     print("Error setting pose:", result.stderr)

        ## PUBLISH MODEL STATE SERVICE REQUEST
        self.pausePhysics(pause_flag=True)
        #self.callService('/gazebo/set_model_state',state_srv,SetModelState)
        self._iterStep(1000)

        # ## SET DESIRED VEL IN CONTROLLER
        # self.sendCmd('GZ_Const_Vel_Traj',cmd_vals=[np.nan,vel[0],0.0],cmd_flag=0.0)
        # self._iterStep(2)
        # #self.sendCmd('GZ_Const_Vel_Traj',cmd_vals=[np.nan,vel[1],0.0],cmd_flag=1.0)
        # self.sendCmd('GZ_Const_Vel_Traj',cmd_vals=[np.nan,0.0,0.0],cmd_flag=1.0)
        # self._iterStep(2)
        # self.sendCmd('GZ_Const_Vel_Traj',cmd_vals=[np.nan,vel[2],0.0],cmd_flag=2.0)
        # self._iterStep(2)
        # self.sendCmd('Activate_traj',cmd_vals=[1.0,1.0,1.0])

        time.sleep(1)
        # ## SET DESIRED VEL IN CONTROLLER
        self.sendCmd('Const_Vel_traj',cmd_vals=[vel[0],self.TrajAcc_Max[0],self.TrajJerk_Max[0]],cmd_flag=0.0)
        self._iterStep(2)
        #self.sendCmd('Const_Vel_traj',cmd_vals=[np.nan,vel[1],0.0],cmd_flag=1.0)
        #self.sendCmd('Const_Vel_traj',cmd_vals=[np.nan,0.0,0.0],cmd_flag=1.0)
        self._iterStep(2)
        self.sendCmd('Const_Vel_traj',cmd_vals=[vel[2],self.TrajAcc_Max[2],self.TrajJerk_Max[2]],cmd_flag=2.0)
        self._iterStep(2)
        self.sendCmd('Activate_traj',cmd_vals=[1.0,1.0,1.0])

        ## ROUND OUT TO 10 ITER STEPS (0.01s) TO MATCH 100Hz CONTROLLER 
        self._iterStep(4)
        self._iterStep(round(t_total * 1000))
        #print("!!!!!!!!!!!!!!!!!!!!! vel :", vel)

    def resetPose(self,z_0=0.4): 
        print("resetPose is started")
        #!!! Need to Change

        self.sendCmd('GZ_StickyPads',cmd_vals=[1.0,1.0,1.0],cmd_flag=0.0)
        #self._iterStep(10)
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
            '--req', f'name: "A30_L200", position: {{x: 0, y: 0, z: {z_0}}}, orientation: {{x: 0, y: 0, z: 0, w: 1}}'
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

        self.sendCmd('GZ_StickyPads',cmd_vals=[1.0,1.0,1.0],cmd_flag=1.0)
        self._iterStep(10)
        self.sendCmd('GZ_StickyPads',cmd_vals=[1.0,1.0,1.0],cmd_flag=0.0)
        self._iterStep(10)
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
            '--req', f'name: "A30_L200", position: {{x: 0, y: 0, z: {z_0}}}, orientation: {{x: 0, y: 0, z: 0, w: 1}}'
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

        self.sendCmd('GZ_StickyPads',cmd_vals=[1.0,1.0,1.0],cmd_flag=1.0)
        
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
        monitor_thread = Thread(target=self._monitor_subprocesses)
        monitor_thread.daemon = True
        monitor_thread.start()

    def _monitor_subprocesses(self):

        while True:

            #self.GZ_ping_ok = self._ping_service("/SAR_DC/CMD_Input",timeout=10)
            
            time.sleep(0.5)

    def _wait_for_sim_running(self,timeout=600):
        #!!! Need to change
        time.sleep(3)

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

    def _restart_Sim(self):
        self.Clock_Check_Flag.clear()

        ## LAUNCH GAZEBO
        self._launch_GZ_Sim()

        # if rospy.get_param(f"/SIM_SETTINGS/GUI_Flag") == True:
        #     self._wait_for_node(node_name="gazebo_gui",timeout=60,interval=2)
        # else:
        #     self._wait_for_node(node_name="gazebo",timeout=60,interval=2)

        ## LAUNCH CONTROLLER
        # self._launch_Controller()
        # self._wait_for_node(node_name="SAR_Controller_Node",timeout=5,interval=0.25)

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

    def callService(self, srv_addr, srv_msg, srv_type, num_retries=5, call_timeout=5):
        """
        Asynchronously call the service and handle the response.
        """
        client = self.create_client(srv_type, srv_addr)

        # Wait until the service is available.
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f"[WARNING] Service '{srv_addr}' not available.")
            return None

        for retry in range(num_retries):
            self.response = None

            # Send asynchronous service request
            future = client.call_async(srv_msg)

            try:
                # Wait for a response within the specified timeout
                rclpy.spin_until_future_complete(self, future, timeout_sec=call_timeout)

                if future.done():
                    try:
                        response = future.result()
                        return response
                    except Exception as e:
                        self.get_logger().warn(f"[WARNING] Service call failed: {e}")
            except Exception as e:
                self.get_logger().warn(f"[WARNING] Attempt {retry + 1} to call service '{srv_addr}' failed: {e}")

            self.get_logger().warn(f"Retrying service call ({retry + 1}/{num_retries}).")

        # In case all attempts fail
        self.Done = True
        self.get_logger().error(f"Service '{srv_addr}' call failed after {num_retries} attempts.")
        return None


    def pausePhysics(self,pause_flag=True):

        # # With Output Message
        # if pause_flag == True:
        #     cmd = f"gz service -s /world/empty/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: true'"
        #     self.pause_simulation_process = subprocess.Popen(cmd, shell=True)

        # else:
        #     cmd = f"gz service -s /world/empty/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false'"
        #     self.pause_simulation_process = subprocess.Popen(cmd, shell=True)

        # Without Output Message
        if pause_flag:
            cmd = f"gz service -s /world/empty/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: true'"
            self.pause_simulation_process = subprocess.Popen(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:
            cmd = f"gz service -s /world/empty/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false'"
            self.pause_simulation_process = subprocess.Popen(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

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
        env.spin()
    except KeyboardInterrupt:
        pass
    finally:
        env.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    