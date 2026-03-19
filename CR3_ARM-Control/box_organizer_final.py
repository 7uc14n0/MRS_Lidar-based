#!/usr/bin/env python3

# --------------------------------------------------------------------------------------
# Developer: Jose Leandro Caires Mirante
# Version: 2.0
# Date: March 19, 2026
# Institution: Universidade Federal de Viçosa (UFV) 
#
# Project: LiDAR-Based Box Dimensioning in a Multi-Robot Logistics Scenario
# Module: Robotic Arm Controller and Pick-and-Place Organizer
#
# Description: This script controls the DOBOT CR3 robotic arm for automated 
#              box sorting and palletization based on LiDAR volume estimation.
#
# License: MIT License 
# --------------------------------------------------------------------------------------

# Library inclusion
import time 
import math 
import copy 
import rclpy 
from RoboticSorter import RoboticSorter 
from Controller_CR3 import Controller_CR3, execute_move_sequence
import csv
from datetime import datetime
import os


# Import constants
from constants import (
    ROBOT_POINTS_DATABASE, 
    POINT_MEASURE, 
    POINT_MEASURE_CARTESIAN, 
    POINT_TAKE_BOX, 
    CAMERA_ROTATION_OFFSET,
    SLOT_POINTS, 
    POINT_CHANGE_SLOT 
)

# --- SCALE AND CALIBRATION CONFIGURATION ---

LIDAR_SCALE_FACTOR = 1000.0 
Z_OFFSET_SAFETY = 65.0
X_OFFSET_FIXED = 100.0 
Y_OFFSET_FIXED = 20.0

# --- HEIGHT CALIBRATION ---
# LiDAR reading for the standard 10cm box
LIDAR_Z_REFERENCE_10CM = 0.26738 

# --- INDIVIDUAL CORRECTION FACTORS ---

# Factor for GOING UP (When the box is LARGER than 10cm)
# If it is going up too much on the large box, decrease to 1.0 or 0.9
# If it is hitting the large box, increase to 1.1 or 1.2
CORRECTION_FACTOR_UP = 1

# Factor for GOING DOWN (When the box is SMALLER than 10cm)
# If it is not going down enough on the small box, increase to 1.2 or 1.3
CORRECTION_FACTOR_DOWN = 1.25   

def apply_z_offset(db_original, offset_z_mm):
    """
    Creates a copy of the points database and applies the Z offset
    only to the CARTESIAN points (Slots and Change Slot).
    """
    db_new = copy.deepcopy(db_original)
    
    # REMOVED POINT_MEASURE FROM HERE
    points_to_adjust = SLOT_POINTS + [POINT_CHANGE_SLOT]
    
    for point_name in points_to_adjust:
        if point_name in db_new:
            original_pose = db_new[point_name]
            # Adds the offset to Z
            original_pose[2] += offset_z_mm
            
            # Safety lock
            LOWER_Z_LIMIT = -235.0 
            if original_pose[2] < LOWER_Z_LIMIT: 
                print(f"WARNING: Point {point_name} reached the lower Z limit!")
                original_pose[2] = LOWER_Z_LIMIT
                
    return db_new

def main(args=None):
    controller_node = None
    connect_to_robot = False
    
    try:
        choice = input("Do you want to connect to the robot? (y/n): ").strip().lower()
        connect_to_robot = (choice == 'y' or choice == 'yes')
        if not connect_to_robot: print("\n*** SIMULATION MODE ***\n")
            
    except KeyboardInterrupt:
        return

    # --- Initialization ---
    if connect_to_robot:
        print("Starting ROS 2...")
        rclpy.init(args=args)
        controller_node = Controller_CR3()
        controller_node.inicialPose()
        
        try:
            input("Press Enter when initial pose is reached...")
            print(">>> Going to Measurement Position...")
            controller_node.move_joints(ROBOT_POINTS_DATABASE.get(POINT_MEASURE))
            time.sleep(3) 
        except KeyboardInterrupt:
            controller_node.destroy_node()
            rclpy.shutdown()
            return
    
    if not rclpy.ok(): rclpy.init(args=args)
    sorter = RoboticSorter()
    sorter.start_messages() 
    
    # Data recording (optional)
    LOG_FILE = "log_robot_sorter_11_Alone.csv"

    if not os.path.exists(LOG_FILE):
        with open(LOG_FILE, mode="w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "timestamp",
                "volume",
                "offset_x","offset_y","offset_z",

                "take_x","take_y","take_z",
                "take_rx","take_ry","take_rz",

                "slot1_x","slot1_y","slot1_z","slot1_rx","slot1_ry","slot1_rz",
                "slot2_x","slot2_y","slot2_z","slot2_rx","slot2_ry","slot2_rz",
                "slot3_x","slot3_y","slot3_z","slot3_rx","slot3_ry","slot3_rz",
                "slot4_x","slot4_y","slot4_z","slot4_rx","slot4_ry","slot4_rz",

                "slot1_volume","slot2_volume","slot3_volume","slot4_volume",
                "change_area_height"
            ])



    # --- Main Loop ---
    try:
        while True:
            print(f"\nCurrent State: {sorter.get_current_state()}")
            sorter.wait_for_start_signal()
            box_size, offsets = sorter.wait_for_volume_reading()
            
            if box_size is None or offsets is None:
                print("Reading error or interruption.")
                continue

            print(f"LiDAR Data (Raw): {offsets}")
            
            # ---------------------------------------------------------
            # 1. DELTA Z CALCULATION WITH SEPARATE FACTORS
            # ---------------------------------------------------------
            current_lidar_z = offsets[2]
            
            # Raw difference in millimeters
            # Positive = Tall Box (Needs to go UP)
            # Negative = Short Box (Needs to go DOWN)
            raw_diff_mm = (LIDAR_Z_REFERENCE_10CM - current_lidar_z) * 1000.0
            
            delta_z_mm = 0.0
            
            # Applies the correct factor depending on the direction
            if raw_diff_mm > 0:
                # GOING UP Case (Large Box)
                delta_z_mm = raw_diff_mm * CORRECTION_FACTOR_UP
                print(f"   Direction: UP (Factor: {CORRECTION_FACTOR_UP})")
            else:
                # GOING DOWN Case (Small Box)
                delta_z_mm = raw_diff_mm * CORRECTION_FACTOR_DOWN
                print(f"   Direction: DOWN (Factor: {CORRECTION_FACTOR_DOWN})")
            
            print(f"--- Variable Height Adjustment ---")
            print(f"   Z Ref (10cm): {LIDAR_Z_REFERENCE_10CM:.4f}")
            print(f"   Current Z: {current_lidar_z:.4f}")
            print(f"   Raw Diff: {raw_diff_mm:.2f} mm")
            print(f"   Final Delta Z: {delta_z_mm:.2f} mm")

            # 2. CREATES A NEW TEMPORARY DB (Adjusted slots)
            current_points_db = apply_z_offset(ROBOT_POINTS_DATABASE, delta_z_mm)

            # ---------------------------------------------------------
            # 3. TAKE BOX CALCULATION
            # ---------------------------------------------------------
            curr_x = POINT_MEASURE_CARTESIAN[0]
            curr_y = POINT_MEASURE_CARTESIAN[1]
            curr_z = POINT_MEASURE_CARTESIAN[2]
            
            lidar_x = offsets[0] * LIDAR_SCALE_FACTOR
            lidar_y = -1 * offsets[1] * LIDAR_SCALE_FACTOR 
            
            # X/Y Rotation
            theta = math.radians(CAMERA_ROTATION_OFFSET)
            delta_x_base = (lidar_x * math.cos(theta)) - (lidar_y * math.sin(theta))
            delta_y_base = (lidar_x * math.sin(theta)) + (lidar_y * math.cos(theta))
            
            # TAKE BOX Z calculation using the corrected logic
            z_base_10cm = (curr_z - (LIDAR_Z_REFERENCE_10CM * 1000.0)) + Z_OFFSET_SAFETY
            new_z = z_base_10cm + delta_z_mm
            
            new_x = curr_x + delta_x_base + X_OFFSET_FIXED
            new_y = curr_y + delta_y_base + Y_OFFSET_FIXED
            
            if new_z < 5.0: new_z = 5.0
            
            calculated_pose = [new_x, new_y, new_z, POINT_MEASURE_CARTESIAN[3], POINT_MEASURE_CARTESIAN[4], POINT_MEASURE_CARTESIAN[5]]
            
            print(f"--- Calculation Report ---")
            print(f"   Final Target (Take Box): {calculated_pose}")
            
            current_points_db[POINT_TAKE_BOX] = calculated_pose
            
            take_x, take_y, take_z, take_rx, take_ry, take_rz = calculated_pose
            
            # ---------------------------------------------
            moves = sorter.add_box(box_size)
            
            # Data recording
            print("Recording data to log...")
            timestamp = datetime.now().isoformat()
            
            slot_poses = []
            for slot_name in SLOT_POINTS:
                pose = current_points_db[slot_name]
                slot_poses.append(pose)  # [x, y, z, rx, ry, rz]


            with open(LOG_FILE, mode="a", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    timestamp,
                    box_size,
                    offsets[0], offsets[1], offsets[2],

                    take_x, take_y, take_z,
                    take_rx, take_ry, take_rz,

                    slot_poses[0][0], slot_poses[0][1], slot_poses[0][2],
                    slot_poses[0][3], slot_poses[0][4], slot_poses[0][5],

                    slot_poses[1][0], slot_poses[1][1], slot_poses[1][2],
                    slot_poses[1][3], slot_poses[1][4], slot_poses[1][5],

                    slot_poses[2][0], slot_poses[2][1], slot_poses[2][2],
                    slot_poses[2][3], slot_poses[2][4], slot_poses[2][5],

                    slot_poses[3][0], slot_poses[3][1], slot_poses[3][2],
                    slot_poses[3][3], slot_poses[3][4], slot_poses[3][5],

                    sorter.slots[0],
                    sorter.slots[1],
                    sorter.slots[2],
                    sorter.slots[3],
                    sorter.change_height if hasattr(sorter, "change_height") else 0.0
                ])

            
            execute_move_sequence(
                moves_list=moves, 
                controller_node=controller_node, 
                points_db=current_points_db, 
                test_mode=not connect_to_robot,
                sorter_node=sorter 
            )

    except KeyboardInterrupt:
        print("\nProgram interrupted.")

if __name__ == '__main__':
    main()