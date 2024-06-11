#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sar_msgs.srv import SetTrigger
import asyncio

class ControlPlayground(Node):
    def __init__(self):
        super().__init__('control_playground')
        self.get_logger().info('Control Playground node has been started.')
        self.done = False

    async def call_service(self, data, vector=None, num_retries=5):
        client = self.create_client(SetTrigger, '/set_trigger')
        
        # CHECK THAT SERVICE IS AVAILABLE
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f"[WARNING] Service '/set_trigger' not available")
            return None

        request = SetTrigger.Request()
        request.data = data
        if vector is not None:
            request.vector = vector  # If vector is existed, set the vector

        # CALL SERVICE AND RETURN RESPONSE
        for retry in range(num_retries):
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

            if future.done():
                try:
                    response = future.result()
                    self.get_logger().info(f'Service call succeeded: {response.success}')
                    return response
                except Exception as e:
                    self.get_logger().warn(f"[WARNING] Attempt {retry + 1} to call service '/set_trigger' failed: {e}")

        # IF SERVICE CALL FAILS THEN MARK SIM EPISODE AS DONE AND RETURN ERROR
        self.done = True
        self.get_logger().error(f"Service '/set_trigger' call failed after {num_retries} attempts")
        return None

async def main(args=None):
    rclpy.init(args=args)
    node = ControlPlayground()
    
    try:
        while rclpy.ok():
            command = input("Enter command 0:Ctrl_Reset, \t1:Turn_Off, \t2:Trigger, \t3:Position Control, \t10:P2P_traj, \t11:Vel_traj")
            if command == '0':
                await node.call_service(0)
            elif command == '1':
                await node.call_service(1)
            elif command == '2':
                await node.call_service(2)
            elif command == '3':
                vector = input("Enter vector values [ex) 0, 0, 0]: ")
                vector = list(map(float, vector.split(',')))
                await node.call_service(3, vector=vector)
            elif command == '10':
                vector = input("Desired positon (x,y,z): [ex) 0, 0, 0]: ")
                vector = list(map(float, vector.split(',')))
                await node.call_service(10, vector=vector)
            elif command == '11':    
                vector = input("Desired Flight Velocity (V_B_O_mag,V_B_O_angle, 0): ")
                vector = list(map(float, vector.split(',')))
                await node.call_service(11, vector=vector)
            else:
                print("Invalid command. Please enter 0 or 1 or 2 or 3 or 10 or 11.")
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())