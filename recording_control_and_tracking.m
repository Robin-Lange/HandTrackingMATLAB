% This script performs hand tracking and drone control using pre-recorded Lidar data.
% It reads point cloud data from a PCAP file, processes the data to identify
% the hand's position and orientation, and visualizes the results.

clear; clc; close all;
warning('off','all');

%% 1. Point Cloud Parameters
% Define the region of interest (ROI) for pattern search.
% The ROI is a 3D box defined by [xmin xmax ymin ymax zmin zmax].
roi = [-.5 -.2 -0.15 0.15 -0.15 0.15];
figure;

%% 2. Data Loading and Initialization
% Define the file names for the Ouster lidar data and metadata.
pcapFile = "OS-0-128_122313000402_2048x10_20250306_183057.pcap";
jsonFile = "OS-0-128_122313000402_2048x10_20250306_183057.json";
% Set the horizontal resolution of the sensor.
horizontalResolution = 2048;
% Create lidar parameters object.
params = lidarParameters('OS0-128', horizontalResolution, SweepStartAngle=135);
% Create an ousterFileReader object to read the PCAP file.
ousterReader = ousterFileReader(pcapFile, jsonFile);
% Set the start time for reading the data.
ousterReader.CurrentTime = ousterReader.StartTime + seconds(0.3);
% Create a cell array to store the point clouds.
storage = {};
% Initialize a counter for storing the data.
i = 1;

% Loop through the PCAP file and store each frame in the storage cell array.
while hasFrame(ousterReader)
    % Read a frame from the ousterReader.
    ptCloudS = readFrame(ousterReader);
    % Store the point cloud in the cell array.
    storage{i} = ptCloudS;
    % Increment the counter.
    i = i + 1;
end

%% 3. Main Loop
p_ref = 0; % Initialize reference position for the hand.
try
    % Loop through each stored point cloud.
    for j = 1:length(storage)
        % Get the current point cloud.
        ptCloud = storage{1, j};

        % Extract the points within the ROI.
        segmented_ptCloud = pointextractor(ptCloud, roi);

        % If no hand is detected, skip to the next iteration.
        if isempty(segmented_ptCloud.Location)
            disp('No hand detected');
            continue;
        end

        % Denoise the point cloud.
        clean_ptCloud = pcdenoise(segmented_ptCloud);

        % In the first iteration, set the initial reference position of the hand.
        if j == 1
            [T, p, ~, ~, ~] = affine_fit(clean_ptCloud.Location);
            if ~isempty(p)
                p_ref = p(1);
            end
        else
            % In subsequent iterations, just get the current position and orientation.
            [T, p, ~, ~, ~] = affine_fit(clean_ptCloud.Location);
        end

        % Process the point cloud to get the control inputs.
        processAndControl(ptCloud, [], p_ref, true);

        %% 4. Visualize
        % Visualize the results.
        visualize_results(clean_ptCloud, p, T, roi);

        % Pause for a short duration to control the loop rate.
        pause(0.5);
    end
catch ME
    % If an error occurs, display the error message.
    fprintf('An error occurred: %s\n', ME.message);
end

disp('Analysis Complete');