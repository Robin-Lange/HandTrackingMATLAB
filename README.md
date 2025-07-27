# HandTrackingMATLAB

"build_rev":"v3.0.1"

This project uses a Lidar sensor to track a hand and control a drone based on the hand's movements.

## Prerequisites

*   MATLAB R2022b or later
*   Lidar Toolbox
*   UAV Toolbox
*   Ryze Tello Drone

## Files

*   `live_control_and_tracking.m`: Main script for real-time drone control.
*   `recording_control_and_tracking.m`: Script for drone control using pre-recorded Lidar data.
*   `processAndControl.m`: Function that processes the point cloud and sends control commands to the drone.
*   `pointextractor.m`: Function that extracts a region of interest from a point cloud.
*   `affine_fit.m`: Function that fits a plane to a set of 3D points.
*   `visualize_results.m`: Function that visualizes the point cloud, the fitted plane, and the normal vector.

## How to Run

### Live Control

1.  Connect the Lidar sensor to your computer.
2.  Connect to the Ryze Tello drone's Wi-Fi network.
3.  Run the `live_control_and_tracking.m` script in MATLAB.

### Recorded Data

1.  Make sure you have the `.pcap` and `.json` files in the project directory.
2.  Run the `recording_control_and_tracking.m` script in MATLAB.

## Suggestions for Improvement

This codebase has been improved by:

1.  **Code organization and modularity:** The main scripts have been broken down into smaller, more manageable functions.
2.  **Error handling and robustness:** The code has been made more robust by adding more error checking and handling.
3.  **Performance:** The performance of the point cloud processing pipeline has been improved by making the denoising step optional.
4.  **Readability and documentation:** The code has been made more readable and easier to understand by adding more comments and documentation.