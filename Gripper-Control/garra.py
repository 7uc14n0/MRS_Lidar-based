# --------------------------------------------------------------------------------------
# Developer: Jose Leandro Caires Mirante
# Version: 2.0
# Date: March 19, 2026
# Institution: Universidade Federal de Viçosa (UFV) 
#            
# Project: LiDAR-Based Box Dimensioning in a Multi-Robot Logistics Scenario
# Module: Gripper Relay Controller (Raspberry Pi GPIO)
#
# Description: This ROS 2 node controls the CR3 robot gripper via a relay.
#              It uses RPi.GPIO to handle a specific hardware issue where
#              residual voltage (1.7V) prevented the relay from turning off.
#              By switching the pin to INPUT (High Impedance), we cut the 
#              current completely.
# --------------------------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO  # Native library for full hardware control
import time

# Define GPIO pin (BCM numbering). 
# CONFIRM IF IT IS 17 (Physical Pin 11) OR 4 (Physical Pin 7)
RELAY_PIN = 17       

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        
        # --- GPIO CONFIGURATION ---
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Initial State: Set as INPUT (High Impedance) to ensure it starts OFF
        GPIO.setup(RELAY_PIN, GPIO.IN)
        # -------------------------

        self.subscription = self.create_subscription(
            String,
            '/garra_command',
            self.listener_callback,
            10)
        
        self.get_logger().info(f'Gripper Node started on pin {RELAY_PIN}. Waiting for commands...')

    def listener_callback(self, msg):
        command = msg.data.lower()
        
        # Logic follows the strings defined in constants.py
        if command == 'abrir':
            self.open_gripper()
        elif command == 'fechar':
            self.close_gripper()
        else:
            self.get_logger().warn(f'Unknown command received: {command}')

    def open_gripper(self):
        self.get_logger().info('Executing: OPEN GRIPPER (Relay ON)')
        # To turn ON: Set as OUTPUT and drive to 0V (GND)
        # (Standard Active Low logic for common relay modules)
        GPIO.setup(RELAY_PIN, GPIO.OUT)
        GPIO.output(RELAY_PIN, GPIO.LOW)
        self.get_logger().info('Status: Relay Activated.')

    def close_gripper(self):
        self.get_logger().info('Executing: CLOSE GRIPPER (Relay OFF)')
        # To turn OFF: Set as INPUT.
        # This "cuts" the wire electronically, removing the 1.7V residual voltage
        # that was keeping the relay stuck in the ON position.
        GPIO.setup(RELAY_PIN, GPIO.IN) 
        self.get_logger().info('Status: Relay Deactivated.')

def main(args=None):
    rclpy.init(args=args)
    gripper_controller = GripperController()
    try:
        rclpy.spin(gripper_controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure pins are cleaned up when the node is killed
        gripper_controller.get_logger().info('Shutting down node and cleaning up GPIO...')
        GPIO.cleanup()
        gripper_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()