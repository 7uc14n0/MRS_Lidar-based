# --------------------------------------------------------------------------------------
# Developer: Jose Leandro Caires Mirante
# Version: 2.0
# Date: March 19, 2026
# Institution: Universidade Federal de Viçosa (UFV) 
#
# Project: LiDAR-Based Box Dimensioning in a Multi-Robot Logistics Scenario
# Module: Robotic Sorter Logic Node
#
# Description: This module implements the sorting logic for the robotic arm. 
#              It manages slot occupancy, determines the sequence of movements 
#              for sorting boxes by size, and handles ROS 2 communication for 
#              volume data and gripper commands.
#
# License: MIT License
# --------------------------------------------------------------------------------------

import select
import sys
import rclpy 
from rclpy.node import Node 
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from constants import (
    POINT_MEASURE, POINT_TAKE_BOX, POINT_ADJUST, POINT_CHANGE_ABOVE, 
    POINT_SLOT_1, POINT_SLOT_2, POINT_SLOT_3, POINT_SLOT_4, 
    POINT_CHANGE_SLOT, SLOT_POINTS, ACTION_GRIP, ACTION_RELEASE, CTRL_START
)

class RoboticSorter(Node):
    def __init__(self):    
        super().__init__('robotic_sorter')
        self.slots = [0.0, 0.0, 0.0, 0.0]
        self.change_area = 0.0
        
        # Detection variables
        self.detected_volume = None 
        self.detected_offsets = [0.0, 0.0, 0.0] 
        self.new_volume_ready = False
        self.send_start_signal = True
        
        print("Robotic Sorter initialized. Slots: [0.0, 0.0, 0.0, 0.0]\n")
        
    def start_messages(self):
        # Subscriptions
        self.msgReceiver = self.create_subscription(String, '/volume_request', self.start_callback, 10)
        self.volume = False
        self.msgVolume = self.create_subscription(Float64MultiArray, '/volume_topic', self.volume_callback, 10)
        
        # Publishers
        self.msgPublisher = self.create_publisher(String, '/string', 10)
        self.msgGarra = self.create_publisher(String, '/garra_command', 10)
        
        self.seq_msg = 0
        self.seq_msg_processed = 0
            
    def wait_for_start_signal(self):
        self.volume = False 
        self.new_volume_ready = False 
        self.detected_volume = None
        
        self.get_logger().info("--- Waiting for 'measure' command (or Press ENTER)... ---")
        
        while (rclpy.ok() and (self.seq_msg_processed == self.seq_msg)):
            rclpy.spin_once(self, timeout_sec=0.1)
            i, o, e = select.select([sys.stdin], [], [], 0)
            if (i):
                sys.stdin.readline()
                self.get_logger().info("Manual Override: Enter pressed by user!")
                break 
        
        self.seq_msg_processed = self.seq_msg 
        self.get_logger().info("Start signal received.")

    def wait_for_volume_reading(self):
        self.get_logger().info("--- Waiting for LIDAR data (/volume_topic)... ---")
        self.volume = True 
        while rclpy.ok() and not self.new_volume_ready:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.volume = False 
        self.new_volume_ready = False
        return self.detected_volume, self.detected_offsets

    def start_callback(self, msg):
        if msg.data == "measure":
            self.seq_msg += 1
            self.get_logger().info("External command received: Measure!")
            self.send_start_signal = False
        else:
            self.get_logger().info(f"Command received: {msg.data}")
            
    def volume_callback(self, msg):
        if self.volume:
            try:
                if len(msg.data) >= 4:
                    raw_vol = msg.data[0]
                    off_x = msg.data[1]
                    off_y = msg.data[2]
                    off_z = msg.data[3]
                    
                    self.get_logger().info(f"LIDAR: Vol={raw_vol:.2f} | Offset=[{off_x:.2f}, {off_y:.2f}, {off_z:.2f}]")
                    self.detected_volume = raw_vol
                    self.detected_offsets = [off_x, off_y, off_z]
                    self.new_volume_ready = True 
                else:
                    self.get_logger().warn(f"Invalid array. Length: {len(msg.data)}")
            except ValueError:
                self.get_logger().error("Invalid value received.")
            
    def get_current_state(self):
        return f"Current Slots: {[round(x, 2) for x in self.slots]}"

    # =========================================================================
    # --- HELPER FUNCTIONS ---
    # =========================================================================

    def _get_new_box(self):
        # NOTE: Removed the first POINT_MEASURE because the robot is already there
        return [POINT_TAKE_BOX, ACTION_GRIP, POINT_MEASURE]

    def _place_in_slot(self, slot_index):
        return [POINT_ADJUST, SLOT_POINTS[slot_index], ACTION_RELEASE, POINT_ADJUST]

    def _place_in_change_area(self):
        return [POINT_CHANGE_ABOVE, POINT_CHANGE_SLOT, ACTION_RELEASE, POINT_CHANGE_ABOVE]

    def _get_from_change_area(self):
        return [POINT_CHANGE_ABOVE, POINT_CHANGE_SLOT, ACTION_GRIP, POINT_CHANGE_ABOVE]

    def _move_box_between_slots(self, from_index, to_index):
        return [
            POINT_ADJUST, 
            SLOT_POINTS[from_index], 
            ACTION_GRIP, 
            POINT_ADJUST, 
            SLOT_POINTS[to_index], 
            ACTION_RELEASE,
            POINT_ADJUST
        ]

    # =========================================================================

    def add_box(self, new_box_size):
        if not isinstance(new_box_size, (int, float)) or new_box_size <= 0:
            print(f"  [ERROR: Invalid box size: {new_box_size}. Must be positive.]")
            return [POINT_ADJUST, POINT_MEASURE]

        print(f"--- Processing new box of size: {new_box_size:.4f} ---")
        movements = []
        
        # 1. PICK UP BOX (Starts directly by going to the take point)
        movements.extend(self._get_new_box())

        insert_index = -1
        for i in range(len(self.slots)):
            if self.slots[i] == 0.0:
                insert_index = i
                break
            if new_box_size < self.slots[i]:
                insert_index = i
                break
        
        if insert_index == -1:
            print("  [Logic: Slots full or box too large.]")
            movements.extend(self._place_in_change_area())
            return movements

        if self.slots[insert_index] == 0.0:
            print(f"  [Logic: Simple insertion at slot {insert_index + 1}]")
            movements.extend(self._place_in_slot(insert_index))
            self.slots[insert_index] = new_box_size
            # The last point in the sequence must be MEASURE to restart the cycle
            movements.append(POINT_MEASURE)  
            return movements

        print(f"  [Logic: Shuffle required at slot {insert_index + 1}]")
        if self.slots[3] != 0.0:
            print("  [Logic: ERROR! Cannot shuffle, the last slot is full.]")
            movements.extend(self._place_in_change_area())
            return movements
        
        movements.extend(self._place_in_change_area())
        self.change_area = new_box_size
        
        last_occupied_index = -1
        for i in range(len(self.slots) - 1, -1, -1):
            if self.slots[i] != 0.0:
                last_occupied_index = i
                break
        
        for i in range(last_occupied_index, insert_index - 1, -1):
            movements.extend(self._move_box_between_slots(i, i + 1))
            self.slots[i + 1] = self.slots[i]
            self.slots[i] = 0.0
            
        movements.extend(self._get_from_change_area())
        self.change_area = 0.0
        
        movements.extend(self._place_in_slot(insert_index))
        self.slots[insert_index] = new_box_size
        
        movements.append(POINT_MEASURE)  
        return movements