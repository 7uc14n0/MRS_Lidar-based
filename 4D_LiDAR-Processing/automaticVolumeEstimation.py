# --------------------------------------------------------------------------------------
# Developer: Luciano Moreira
# Version: 3.0
# Date: March 19, 2026
# Institution: Federal University of Viçosa (UFV) /
# 		Federal Institute of Southeast Minas Gerais (IF Sudeste MG)
#
# Project: ROS 2-based Multi-Robot System (MRS) for Intralogistics
# Module: LiDAR-based 3D Volume Estimation and Object Segmentation
#
# Description: This code accumulates the point cloud from a LiDAR in real-time and displays it in a 3D window using Open3D.
#		Then, the code identifies the object to be measured, segments the object, and calculates its volume. 
#		Finally, the code displays the accumulated point cloud, the bounding box of the identified object, the object's measurements, 
#		and the estimated volume. Additionally, it displays the execution time and saves the accumulated cloud in a PCD file.
# License: MIT License (or your preferred license)
# --------------------------------------------------------------------------------------
import time # Importing the time library to measure execution time
import numpy as np # Importing the NumPy library for array manipulation
import open3d as o3d    # Importing the Open3D library for 3D point cloud visualization and manipulation
import rclpy # Importing the rclpy library to work with ROS 2
from rclpy.node import Node # Importing the Node class from rclpy to create a ROS node
from sensor_msgs.msg import PointCloud2 # Importing the PointCloud2 message from ROS to handle point clouds
import struct   # Importing the struct library to unpack binary data
from pathlib import Path    # Importing Path from the pathlib library for file path manipulation
import csv  # Importing the csv library for CSV file manipulation
import os # Importing the os library for file and directory manipulation
from std_msgs.msg import String, Float64MultiArray # Importing standard ROS messages

start_time = 0   # Starting the timer to measure execution time
# Global variables for trial and success control
tests = 0 # Counter for LiDAR data collection tests
testsOk = 0 # Counter for successful tests
bb_hist = []  # Bounding box history to check for stabilization
cluster_size_history = []   # List to store the size of found clusters
endReadSensor = False
requestVolume = False
volume_list = []  # List to store calculated volumes
total_time = 0  # Variable to accumulate total execution time

# Main ROS node class that subscribes to the PointCloud2 message
class LidarSubscriber(Node):
    # ROS node initialization 
    def __init__(self):
        super().__init__('python_node') # ROS node name
        # Creates and subscribes to the topic (message type, topic, callback function, queue size)
        self.subscription = self.create_subscription(
            PointCloud2,
            '/unilidar/cloud',
            self.callback,
            10
        )

        self.subscription # Subscription variable, necessary to keep the subscription active
        # Subscription to receive volume requests        
        self.subscription_string = self.create_subscription( 
            String,
            '/volume_request',
            self.callback_string,
            10
        )
        self.subscription_string  # Volume request topic subscription variable, necessary to keep the subscription active
       
        # Float64MultiArray message publisher for volume
        self.publisher_volume = self.create_publisher(
            Float64MultiArray, 
            '/volume_topic', 
            10) 
        self.publisher_volume  # Volume publisher variable, necessary to keep the publisher active    

        # Defining the 3D space limits from which points will be considered
        self.z_min = 0.025    # Minimum height of the points
        self.z_max = 0.32    # Maximum height of the points
        self.x_min = -0.25   # Minimum limit of the X axis 
        self.x_max = 1.0    # Maximum limit of the X axis
        self.y_min = -1.0   # Minimum limit of the Y axis
        self.y_max = 1.0    # Maximum limit of the Y axis

        self.accumulated_points = []    # List to accumulate points received from the LiDAR

        # Initializing the Open3D visualizer
        self.vis = o3d.visualization.Visualizer() # Creates an instance of the Open3D visualizer
        self.vis.create_window("Occupancy Grid 3D", width=800, height=600) # Creates a window for 3D visualization
        self.point_cloud = o3d.geometry.PointCloud() # Creates an empty point cloud
        self.geometry_added = False # Flag to check if geometry has already been added to the visualizer
        self.vis.get_render_option().point_size = 0.1 # Point size in the visualization

    # Callback function called when a new String message is received
    def callback_string(self, msg):
        global requestVolume
        print(f"Volume request received: {msg.data}")
        # Here you can add logic to start data collection or process the request as needed
        if msg.data == "measure":
            self.get_logger().info("Starting data collection for volume estimation.")  
            requestVolume = True
        else:
            self.get_logger().info("Unknown volume request message.")
            requestVolume = False

    # Callback function called when a new PointCloud2 message is received
    def callback(self, msg):
        new_points = self.convert_pointcloud2_to_numpy(msg) # Calls function that converts the PointCloud2 message to a NumPy array
        # Filters points based on defined limits and keeps only the points within the defined limits
        new_points = new_points[(new_points[:, 0] >= self.x_min) & (new_points[:, 0] <= self.x_max) &
                                (new_points[:, 1] >= self.y_min) & (new_points[:, 1] <= self.y_max) &
                                (new_points[:, 2] >= self.z_min) & (new_points[:, 2] <= self.z_max)]
        self.accumulated_points.extend(new_points) # Adds the new filtered points to the accumulated points list

        self.point_cloud.points = o3d.utility.Vector3dVector(np.array(self.accumulated_points)) # Creates an Open3D point cloud from the accumulated points
        
        # Checks if the geometry has already been added to the visualizer; if so, updates the geometry, otherwise adds it
        if not self.geometry_added:
            self.vis.add_geometry(self.point_cloud) # Adds the point cloud to the visualizer
            self.geometry_added = True # Marks that the geometry has been added
        else:
            self.vis.update_geometry(self.point_cloud) # Updates the point cloud geometry in the visualizer
        self.vis.poll_events()  # Processes visualizer events
        self.vis.update_renderer() # Updates the visualizer's renderer

    # Function to convert the PointCloud2 message into a NumPy array
    def convert_pointcloud2_to_numpy(self, msg):
        point_step = msg.point_step # Size of each point in the message
        data = msg.data # Raw data of the PointCloud2 message
        points = [] # List to store the converted points
        for i in range(0, len(data), point_step):
            try:    # Tries to unpack the binary data to obtain x, y, z coordinates
                x, y, z = struct.unpack_from('fff', data, i)    # Unpacks the binary data to obtain the x, y, z coordinates
                points.append([x, y, z])    # Adds the x, y, z coordinates to the points list
            except struct.error:    # Handles unpacking errors in case the data is incomplete
                continue        # Ignores points with incomplete data and restarts the loop
        return np.array(points) # Returns the points as a NumPy array
    
    # Function to detect and segment the object in the accumulated point cloud
    def detect_and_segment_object(self):
       # ================ Auxiliary functions =========================
       # Function to check if the bounding box has stabilized
       def bounding_box_stable(pcd, bb_hist, delta_thresh=0.03, window=3): 
            
            # Calculates current bounding box
            bbox = pcd.get_minimal_oriented_bounding_box()
            extent = bbox.extent  # [width_x, height_y, height_z]
            
            dim1, dim2 = extent[0], extent[1]
            length = max(dim1, dim2)
            width = min(dim1, dim2) 

            # Adds to history
            bb_hist.append((width, length))

            # Ensures maximum window size
            if len(bb_hist) > window:
                bb_hist.pop(0)

            # If we still don't have enough history, it hasn't stabilized
            if len(bb_hist) < window:
                return False

            # Checks the variation in the latest measurements
            widths = [w for w, l in bb_hist]
            lengths = [l for w, l in bb_hist]

            max_w, min_w = max(widths), min(widths)
            max_l, min_l = max(lengths), min(lengths)

            # If the variations are within the threshold, we consider it stabilized
            if (max_w - min_w) < delta_thresh and (max_l - min_l) < delta_thresh:
                return True

            return False
       
       # Function to calculate OBB dimensions and volume
       def calculate_Volume_Dimensions(obb):
            volume = 0
            length = 0.0 # Can add a small gain to improve accuracy
            width = 0.0 # Can add a small gain to improve accuracy
            height = 0.0 # Can add a small gain to improve accuracy
            heightSensorGround = 0.35  # Sensor height relative to the ground

            # --- Calculation of the object's height ---
            z_values = np.asarray(pcd.points)[:, 2] # Gets an array of Z values from the point cloud
            objectTop = np.percentile(z_values, 5)  # Gets the 5th percentile of the Z values (adjustable)
            heightObject = heightSensorGround - objectTop  # Calculates the object's height relative to the platform on the ground (10cm)
            heightObject = max(0.01, heightObject)  # Ensures minimum height

            # --- Identification of the OBB Z-axis ---
            z_global = np.array([0, 0, 1])  # Global Z vector pointing downwards
            dot_products = np.abs(np.dot(obb.R.T, z_global)) # Calculates the dot product between the global Z vector and the OBB rotation vectors
            z_index = np.argmax(dot_products) # Finds the index of the rotation vector closest to the global Z

            # --- Calculation of dimensions ---
            xy_indices = [i for i in range(3) if i != z_index] # Indices of the X and Y axes (excluding Z)
            width += min(obb.extent[xy_indices[0]], obb.extent[xy_indices[1]]) # width is the smaller of the two X and Y axes
            length += max(obb.extent[xy_indices[0]], obb.extent[xy_indices[1]]) # length is the larger of the two X and Y axes
            height += heightObject  # Recalculated based on the sensor-top difference

             # Volume calculation
            volume = width * length * height

            # Display results
            print(f"Estimated OBB volume: {obb.volume():.3f} m³")
            print("\nIdentified object dimensions:")
            print(f" length: {length:.3f} m")
            print(f" width:     {width:.3f} m")
            print(f" height:      {height:.3f} m")
            print(f" Volume:      {volume:.6f} m³\n")

            return volume, length, width, height  # returns dimensions
       # =================== End of auxiliary functions =========================

       global testsOk, start_time, bb_hist, endReadSensor, volume_list, total_time  # Indicates that the global variable will be used
       volume = 0
       length = 0
       width = 0
       height = 0
       
       # Start object detection and segmentation
       print("Starting complete object detection with bounding box...")
       # Create point cloud
       pcd = o3d.geometry.PointCloud()
       pcd.points = o3d.utility.Vector3dVector(np.array(self.accumulated_points))

       # Filter: downsampling with voxel grid that reduces cloud density
       voxel_size = 0.01 # cloud density 
       pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

       # Filter: outlier removal that removes isolated points
       pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=50, std_ratio=0.25) 

       # Execute RANSAC
       plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                        ransac_n=3,
                                        num_iterations=2000)

       # Select only inlier points (well-fitted to the plane)
       pcd = pcd.select_by_index(inliers)

       # Estimate point cloud normals
       pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))
       pcd.orient_normals_consistent_tangent_plane(k=30)

       # DBSCAN for cluster segmentation
       labels = np.array(pcd.cluster_dbscan(eps=0.05, min_points=30, print_progress=False)) 
       max_label = labels.max()
       print(f"{max_label + 1} clusters found.")

       if max_label < 0:
           print("No clusters found.")
           return

       clusters = []
       centers = []

       for i in range(max_label + 1):
           indices = np.where(labels == i)[0]
           cluster_points = np.asarray(pcd.points)[indices]
           if len(cluster_points) >= 200: #500 # minimum cluster size
               clusters.append(cluster_points)
               centers.append(np.mean(cluster_points, axis=0))

       if not clusters:
           print("No clusters with sufficient size.")
           return

       # Find the largest cluster
       cluster_sizes = [len(c) for c in clusters]
       largest_idx = np.argmax(cluster_sizes)
       largest_center = centers[largest_idx]

       # Combine clusters close to the largest one
       merged_points = clusters[largest_idx]
       for i, cluster in enumerate(clusters):
           if i == largest_idx:
               continue
           dist = np.linalg.norm(np.array(centers[i]) - np.array(largest_center))
           if dist < 10.0:  # proximity tolerance (adjustable) in mm
               merged_points = np.vstack((merged_points, cluster))

       print(f"Object identified with {merged_points.shape[0]} points.")

       # Create point cloud and OBB
       object_pcd = o3d.geometry.PointCloud()
       object_pcd.points = o3d.utility.Vector3dVector(merged_points)

       if bounding_box_stable(object_pcd, bb_hist):
            print("Bounding box stabilized. Acquisition can be terminated.")
            endReadSensor=True

       obb = object_pcd.get_minimal_oriented_bounding_box() # Minimum bounding box
             
       # Create reference axes
       sensor_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])

       # Calculate volume and dimensions
       volume, length, width, height = calculate_Volume_Dimensions(obb)

       # End execution time
       end_time = time.time()
       elapsed_time = end_time - start_time
       total_time += elapsed_time
       print(f"Total execution time: {total_time:.2f} seconds")

       # Check if the sensor reading was successfully completed
       if endReadSensor: 
            print("Successful attempt! Volume within standard.") 
            testsOk += 1  # increments the successful attempts counter           

            print("Volume successfully stabilized.")

            # Directory creation (only needs to be done once)
            output_dir = Path.home() / "PCDs/BoundingBoxVolume/TestesMRS_9/Teste_2" # Defines the output directory
            output_dir.mkdir(parents=True, exist_ok=True)

            # Adds the attempt number to the file names
            num = testsOk  # Successful attempt number
            # Paths to save the files
            pcd_path = output_dir / f"accumulated_cloud_{num}.pcd"
            obb_path = output_dir / f"obb_{num}.ply"
            csv_path = output_dir / f"volume_result_{num}.csv"

            # Save point cloud and OBB
            o3d.io.write_point_cloud(str(pcd_path), pcd)
            print(f"Accumulated point cloud saved to '{pcd_path}'.")

            # Identifying the OBB centroid coordinates
            center_obb = obb.center # OBB center
            print(f"OBB Center: {center_obb}")

            # Data to save
            dados = {
                "OBB Volume (m³)": round(obb.volume(), 3),
                "length (m)": round(length, 3),
                "width (m)": round(width, 3),
                "height (m)": round(height, 3),
                "Calculated Volume (m³)": round(volume, 6),
                "Execution Time": round(elapsed_time, 2)
            }

            # Save individual CSV for each attempt
            with open(csv_path, mode='w', newline='') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=dados.keys())
                writer.writeheader()
                writer.writerow(dados)

            # Send estimated volume via ROS2
            volume_msg = Float64MultiArray()
            volume_msg.data = [volume*1000, obb.center[0], obb.center[1], obb.center[2]]  # Converting m³ to liters
            self.publisher_volume.publish(volume_msg)
            print(f"Estimated volume = {volume*1000} published to 'volume_topic' topic.")

            print(f"Results saved to: {csv_path}")

            # --- Temporary visualization ---
            o3d.visualization.draw_geometries(
                [pcd, obb, sensor_frame],
                    window_name="Identified Object OBB",
                )

def main():
    global tests, testsOk, start_time, bb_hist, endReadSensor, requestVolume  # Indicates that the global variables will be used
    tmax = 1.5 # Acquisition time
    rclpy.init()
    node = LidarSubscriber()

    # Waits for volume request message
    print("Waiting for volume request...")
    
    while not requestVolume:
        # rclpy.spin_once(node) executes the ROS 2 event loop once, allowing the node to process received messages.
        rclpy.spin_once(node, timeout_sec=0.1) # Waits 100 ms before checking again

    node.accumulated_points = []  # Clears accumulated points before starting collection

    print("Positioning the robot for data collection...")

    i = 0
    while i<10:  # Short pause before starting data collection
        rclpy.spin_once(node) # Waits 100 ms before checking again
        time.sleep(0.1)  # 1-second pause
        i += 1

    # Data collection loop
    while tests < 10:  # Tries up to 10 times
        print(f"Starting LiDAR data collection attempt {tests + 1}...")
        start_time = time.time()    # Restarts the timer for the next attempt
        while ((time.time() - start_time) < tmax) and not endReadSensor:
            # After collection time, detect and segment the object
            node.detect_and_segment_object()
        tests += 1
        print(f"Attempt {tests} completed.")
        endReadSensor=False
        # Checks if 1 successful attempt was made        
        if testsOk == 1:
            print("1 successful attempt recorded. Terminating the program.")
            break
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()