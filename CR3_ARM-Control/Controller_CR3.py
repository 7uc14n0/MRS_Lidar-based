# --------------------------------------------------------------------------------------
# Developer: Jose Leandro Caires Mirante
# Version: 2.0
# Date: March 19, 2026
# Institution: Universidade Federal de Viçosa (UFV) 
#            
# Project: LiDAR-Based Box Dimensioning in a Multi-Robot Logistics Scenario
# Module: DOBOT CR3 Core Controller
#
# Description: This module provides the ROS 2 node class and execution 
#              functions to control the DOBOT CR3 robotic arm, including 
#              Cartesian and Joint movement interfaces and action sequencing.
#
# License: MIT License 
# --------------------------------------------------------------------------------------

import time 
import math
import rclpy 
from rclpy.node import Node 
from dobot_msgs_v3.srv import GetPose, GetAngle 
from dobot_msgs_v3.srv import JointMovJ, MovJ 
from std_msgs.msg import String

# --- ADD 'POINT_CHANGE_SLOT' HERE IN THE LIST ---
from constants import (
    ROBOT_POINTS_DATABASE, 
    ACTION_GRIP, 
    ACTION_RELEASE, 
    POINT_ADJUST, 
    POINT_TAKE_BOX, 
    SLOT_POINTS, 
    POINT_CHANGE_SLOT 
)

class Controller_CR3(Node):
    def __init__(self):
        super().__init__('Controller_CR3') 
        self.get_logger().info('Connecting to Dobot services...')
        
        self.client_pose = self.create_client(GetPose, '/dobot_bringup_v3/srv/GetPose')
        self.client_angle = self.create_client(GetAngle, '/dobot_bringup_v3/srv/GetAngle')
        self.client_move_joint = self.create_client(JointMovJ, '/dobot_bringup_v3/srv/JointMovJ')
        self.client_move_cartesian = self.create_client(MovJ, '/dobot_bringup_v3/srv/MovJ')
        self.msgReceiver = self.create_subscription(String, '/volume_request', self.start_callback, 10)
        self.enableMeasure = False
        
        while not self.client_pose.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for GetPose service...')
        while not self.client_angle.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for GetAngle service...')
            
        self.get_logger().info('CR3 Controller Ready!')
        
    def start_callback(self, msg):
        if msg.data == "measure":
            self.enableMeasure = True
        else:
            self.enableMeasure = False
        
    def getPose(self):
        req = GetPose.Request()  
        future = self.client_pose.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if hasattr(resp, 'pose') and resp.pose:
            pose_values = resp.pose.strip('{}').split(',')
            return [float(x) for x in pose_values]
        return None

    def getAngle(self):
        req = GetAngle.Request()
        future = self.client_angle.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if hasattr(resp, 'angle') and resp.angle:
            angle_values = resp.angle.strip('{}').split(',')
            return [float(x) for x in angle_values]
        return None
        
    def inicialPose(self):       
        self.move_joints(ROBOT_POINTS_DATABASE.get("INITIAL_POSE"))
        self.wait_for_arrival(ROBOT_POINTS_DATABASE.get("INITIAL_POSE"), is_joint=True)

    def move_joints(self, dPose):
        if not dPose: return
        req  = JointMovJ.Request()
        req.j1, req.j2, req.j3, req.j4, req.j5, req.j6 = dPose
        self.client_move_joint.call_async(req)

    def move_cartesian(self, dPose):
        if not dPose: return
        req = MovJ.Request()
        req.x, req.y, req.z = dPose[0], dPose[1], dPose[2]
        req.rx, req.ry, req.rz = dPose[3], dPose[4], dPose[5]
        self.client_move_cartesian.call_async(req)

    def wait_for_arrival(self, target_vector, is_joint=True, tolerance=2.0, timeout=15.0):
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            current = self.getAngle() if is_joint else self.getPose()
            if not current: continue

            all_good = True
            for i in range(len(target_vector)):
                diff = abs(current[i] - target_vector[i])
                if diff > tolerance:
                    all_good = False
                    break
            if all_good: return True
            time.sleep(0.1)
        self.get_logger().warn("TIMEOUT: Robot did not confirm arrival.")
        return False

def execute_move_sequence(moves_list, controller_node, points_db, test_mode, sorter_node):
    if not moves_list: return

    print("  Executing move sequence...")
    for i, item in enumerate(moves_list):
        
        if item == ACTION_GRIP:
            print(f"    {i+1}. [ACTION] GRAB BOX (Open)")
            msg = String(); msg.data = "abrir" # Maintained in Portuguese for integration with gripper controller
            sorter_node.msgGarra.publish(msg)
            time.sleep(1.0)
            continue

        elif item == ACTION_RELEASE:
            print(f"    {i+1}. [ACTION] RELEASE BOX (Close)")
            msg = String(); msg.data = "fechar" # Maintained in Portuguese for integration with gripper controller
            sorter_node.msgGarra.publish(msg)
            time.sleep(1.0)
            continue
        
        target_pose = points_db.get(item)
        if not target_pose: 
            print(f"ERROR: Point {item} not found in DB.")
            continue

        if test_mode:
            print(f"    {i+1}. [TEST] Going to: {item}")
            time.sleep(0.5)
        else:
            controller_node.get_logger().info(f"Move to: {item}")
            
            # --- NEW DECISION LOGIC ---
            # Added: POINT_CHANGE_SLOT to the list of Cartesian movements
            if item == POINT_TAKE_BOX or item in SLOT_POINTS or item == POINT_CHANGE_SLOT:
                
                # Cartesian Movement (MovJ)
                controller_node.move_cartesian(target_pose)
                # Cartesian Check (X, Y, Z...)
                controller_node.wait_for_arrival(target_pose, is_joint=False, tolerance=5.0)
                
            else:
                # Joint Movement (JointMovJ)
                controller_node.move_joints(target_pose)
                # Joint Check (J1...J6)
                controller_node.wait_for_arrival(target_pose, is_joint=True, tolerance=2.0)
            
        if item == POINT_ADJUST and controller_node.enableMeasure:
            print("    >>> Reached POINT_ADJUST. Triggering 'start'...")
            msg = String(); msg.data = "start"
            sorter_node.msgPublisher.publish(msg)
            controller_node.enableMeasure = False

    print("="*40)