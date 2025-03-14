clear; clc; close all;
%% 3. Point Cloud Processing and Visualization
% Define the initial region of interest (ROI) for pattern search.
roi = [-.5 -.2 -0.15 0.15 -0.15 0.15];
% Define the minimum distance between points for clustering (in meters).
minDistance = 0.009;
% Define the minimum number of points required for a cluster to be considered valid.
minPoints = 1000;
% Initialize the previous centroid to an empty array.
prevCentroid = [];

%% 1. Data Loading and Initialization
% Define the file names for the Ouster lidar data and metadata.
pcapFile = "OS-0-128_122313000402_2048x10_20250306_183057.pcap";
jsonFile = "OS-0-128_122313000402_2048x10_20250306_183057.json";
% Create an ousterFileReader object to read the lidar data.
try
    ousterReader = ousterFileReader(pcapFile, jsonFile);
catch ME
    error("Error creating ousterFileReader. Ensure the PCAP and JSON files exist and are valid. Error: %s", ME.message);
end
storage = {};      % Cell array to store point clouds
i = 1; % Initialize a counter for storing the data.
%% 2. Point Cloud Reading and Storage
while hasFrame(ousterReader)
    % Read a frame from the ousterReader, obtaining the point cloud and
    % associated attributes.
    ptCloud = readFrame(ousterReader);
    % Store the point cloud and near-infrared data in cell arrays.
    storage{i} = ptCloud;
    % Increment the counter.
    i = i + 1;
end

%connect to the drone
droneObj = ryze();
takeoff(droneObj);
%% Loop
% Loop through the stored point clouds, starting from the 1st frame.
for i = 1:length(storage)

    % Find the indices of points within the defined ROI.
    indices = findPointsInROI(storage{1, i}, roi);
    % Select the points within the ROI to create a new point cloud.
    ptCloudB = select(storage{1, i}, indices);


    % Segment the point cloud into clusters based on distance and minimum point count.
    [labels, numClusters] = pcsegdist(ptCloudB, minDistance, NumClusterPoints=minPoints);
    % Find the indices of valid points (points that belong to a cluster).
    idxValidPoints = find(labels);
    % Select the valid points to create a segmented point cloud.
    segmentedPtCloud = select(ptCloudB, idxValidPoints);

    % Check if the segmented point cloud is empty.
    if isempty(segmentedPtCloud.Location)
        disp('No hand detected');
        % Reset the previous centroid.
        prevCentroid = [];
        % Skip to the next iteration.
        continue;
    end
    % Calculate the centroid of the current segmented point cloud.
    currentCentroid = mean(segmentedPtCloud.Location, 1);

    
    % Display the vector
    if ~isempty(prevCentroid)
        direction = currentCentroid - prevCentroid;
        direction_drone = 10*direction;
        if direction_drone(1)>0.2 | direction_drone(2)>0.2 | direction_drone(3)>0.2
            fprintf('Move drone to: [%f, %f, %f]\n', direction_drone);
            move(droneObj,direction_drone);
        else
            %fprintf('Hand didnt move enough: [%f, %f, %f]\n', direction_drone);
            continue;
        end
    end
    % Update the previous centroid.
    prevCentroid = currentCentroid;
end
disp('Landing Requested');
land(droneObj);