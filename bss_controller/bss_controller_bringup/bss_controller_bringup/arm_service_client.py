import rclpy
from rclpy.node import Node

from bss_controller_interface.srv import BSSControl

# BSSControl.srv
# ****************
# int8 index
# int8 row
# int8 col
# int8 scoop_state
# ---
# string result

class ArmServiceClient(Node):
    
    def __init__(self):
        
        super().__init__('arm_service_client')
        
        self._service_client = self.create_client(
            BSSControl,
            'bss_ui_command')
        
        while not self._service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Service...')
        
        self.req = BSSControl.Request()

    def send_request(self, index, row, col, scoop_state):
        
        self.req.index = index
        self.req.row = row
        self.req.col = col
        self.req.scoop_state = scoop_state # Neutral, Catch, Droop
        
        self.future = self._service_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    arm_service_client = ArmServiceClient()
    
    try:
        arm_service_client.get_logger().info('Nodes Ready, BSS Commander Service Client is running. Shut down with CTRL-C')
        
        response = arm_service_client.send_request(1, 0, 0, 1)
        arm_service_client.get_logger().info('Result: ' + response.result)
        
    except KeyboardInterrupt:
        arm_service_client.get_logger().info('Keyboard Interrupt, shutting down.')
    
    arm_service_client.destroy_node()

if __name__ == '__main__':
    main()
    