# --------------------------------------------------------------------------------------
# Developer: Jose Leandro Caires Mirante
# Version: 2.0
# Date: March 19, 2026
# Institution: Universidade Federal de Viçosa (UFV) 
#
# Project: LiDAR-Based Box Dimensioning in a Multi-Robot Logistics Scenario
# Module: System Constants and Points Database
#
# Description: This file stores all movement constants, gripper actions, 
#              calibration offsets, and the coordinate database for the 
#              robotic arm's workspace.
#
# License: MIT License
# --------------------------------------------------------------------------------------

# --- Point Names (MOVEMENT CONSTANTS) ---
POINT_MEASURE = "Go to 'Measure' point (above new box)"
POINT_TAKE_BOX = "Go to 'Take Box' point (grip new box)"
POINT_ADJUST = "Go to 'Adjust' point (above slots)"
POINT_CHANGE_ABOVE = "Go to 'Change Area Above' point"
POINT_SLOT_1 = "Place in 'Slot 1'"
POINT_SLOT_2 = "Place in 'Slot 2'"
POINT_SLOT_3 = "Place in 'Slot 3'"
POINT_SLOT_4 = "Place in 'Slot 4'"
POINT_CHANGE_SLOT = "Place in 'Change Area Slot' (temporary)"

SLOT_POINTS = [POINT_SLOT_1, POINT_SLOT_2, POINT_SLOT_3, POINT_SLOT_4]

# --- Gripper Actions (ACTION CONSTANTS) ---
ACTION_GRIP = "!!! TURNING GRIPPER ON !!!"
ACTION_RELEASE = "!!! TURNING GRIPPER OFF !!!"

# --- Control Variables ---
CTRL_START = False

# --- CARTESIAN CONSTANTS (FIXED) ---
POINT_MEASURE_CARTESIAN = [436.817474, -34.176186, 404.567047, -179.231705, -1.047668, -76.764702]

# --- CAMERA ROTATION CALIBRATION (NEW) ---
# Define the camera rotation angle relative to the robot's wrist here.
# If it were perfect at 90 degrees, it would be 90.0.
# Since the image appears tilted, try adjusting: e.g., 85.0, 95.0, 100.0, etc.
CAMERA_ROTATION_OFFSET = 30.0

# --- POINTS DATABASE (COORDINATES) ---
ROBOT_POINTS_DATABASE = {
    "INITIAL_POSE": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    POINT_MEASURE: [12.610082, -22.007996, -36.182922, -30.354858, 89.204308, -0.461888], 
    POINT_TAKE_BOX: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
    POINT_CHANGE_ABOVE: [-229.1447, -46.2188, -51.2931, 12.3037, 93.3664, -0.0255], 
    POINT_CHANGE_SLOT: [-213.5703,
                        496.9577,
                        -178.7658,
                        176.6808,
                        4.0411,
                        36.0728],
    POINT_ADJUST: [-110.9599, -42.4403, -53.4256, 29.0344, 93.3654, -0.0255],
    POINT_SLOT_1: [179.2547,
                    -531.1418,
                    -178.856,
                    -178.7658,
                    3.3596,
                    -148.6614],
    POINT_SLOT_2: [-77.4851,
                    -555.3969,
                    -178.5499,
                    -176.7322,
                    3.3594,
                    -175.2552],
    POINT_SLOT_3: [-300.5411,
                    -473.4385,
                    -178.5509,
                    -176.7321,
                    3.3595,
                    160.2795],
    POINT_SLOT_4: [-510.434,
                    -232.2213,
                    -158.5502,
                    -176.732,
                    3.3595,
                    127.1501],
}