% This script performs live hand tracking and drone control.
% It uses an Ouster Lidar sensor to capture point cloud data,
% processes the data to identify the hand's position and orientation,
% and sends control commands to a Ryze Tello drone.

clear; clc; close all;
warning('off','all');

%% 1. Point Cloud Parameters
% Define the initial region of interest (ROI) for pattern search.
% The ROI is a 3D box defined by [xmin xmax ymin ymax zmin zmax].
roi = [0.3 0.6 -.15 .15 -.15 .15];
figure;

%% 2. Sensor Initialization and Viewfinder
% Create a lidar object for the Ouster OS0-128 sensor.
ousterObj = ousterlidar("OS0-128","OS-0-128_122313000402.json");
% Set the horizontal resolution of the sensor.
horizontalResolution = 2048;
% Create lidar parameters object.
params = lidarParameters('OS0-128', horizontalResolution, SweepStartAngle=135);
% Preview the lidar data.
preview(ousterObj,[0.3 0.6],[-.15 .15], [0 .30]);
pause(3)
closePreview(ousterObj);
% Start capturing data from the lidar sensor.
start(ousterObj);
pause(3);

%% 3. Drone Initialization
% Create a ryze object for the Tello drone.
droneObj = ryze();
% Create a camera object for the drone's camera.
cameraObj = camera(droneObj);
% Take off the drone.
takeoff(droneObj);
% Take a snapshot from the drone's camera.
frame = snapshot(cameraObj);
% Preview the drone's camera feed.
preview(cameraObj);

%% 4. Main Loop
p_ref = 0; % Initialize reference position for the hand.
try
    % Loop for a fixed number of iterations.
    for j = 1:1000
        % Read the latest point cloud data from the lidar sensor.
        ptCloud = read(ousterObj, 1, "latest");

        % In the first iteration, set the initial reference position of the hand.
        if j == 1
            [~, p, ~, ~, ~] = affine_fit(pointextractor(ptCloud, roi).Location);
            if ~isempty(p)
                p_ref = p(1);
            end
        end

        % Process the point cloud and control the drone.
        processAndControl(ptCloud, droneObj, p_ref, true);

        % Pause for a short duration to control the loop rate.
        pause(0.5);
    end
catch ME
    % If an error occurs, display the error message.
    fprintf('An error occurred: %s\n', ME.message);
end

%% 5. Landing and Cleanup
% Land the drone.
disp('Landing Requested');
land(droneObj);
% Close the camera preview.
closePreview(cameraObj);
% Stop capturing data from the lidar sensor.
stop(ousterObj);