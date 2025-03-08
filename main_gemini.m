%%% Script Description:
% This script processes point cloud data from an Ouster lidar sensor,
% segments the point cloud to identify clusters within a defined region of
% interest (ROI), and calculates the movement direction between consecutive
% point cloud centroids.  The script is designed to potentially interface
% with a drone (using the commented-out `ryze` object) to guide its
% movement based on the calculated direction vectors.
%
% The script reads point cloud data from a PCAP file and its associated JSON
% metadata file. It iterates through the frames, extracts point clouds,
% segments them based on distance and minimum point count, and calculates
% the centroid of the largest cluster. The direction vector between
% consecutive centroids is then computed and displayed.  The script also
% includes commented-out sections for controlling a Ryze drone.
%
% Note: The drone control parts are commented out, as the drone object and
% associated functions (takeoff, move, land) are not defined in the
% provided code.  To use the drone control functionality, you would need to
% uncomment these sections and ensure that the `ryze` object is properly
% initialized and connected to the drone.

clear;      % Clear the workspace
clc;        % Clear the command window
close all;  % Close all open figures

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

% Set the starting time for reading the data.  This skips the first 0.3
% seconds of the recording.
ousterReader.CurrentTime = ousterReader.StartTime + seconds(0.3);

% Initialize variables to store the point clouds and near-infrared data.
storage = {};      % Cell array to store point clouds
nearIr = {};       % Cell array to store near-infrared data

% % Connect to the drone (commented out).  Uncomment this section and
% % initialize the drone object if you want to control a Ryze drone.
% try
%     droneObj = ryze();
% catch ME
%     warning("Could not connect to the drone. Drone control functionality will be disabled. Error: %s", ME.message);
%     droneObj = []; % Set droneObj to empty to avoid errors later
% end

i = 1; % Initialize a counter for storing the data.

%% 2. Point Cloud Reading and Storage

% Loop through the lidar data frames until there are no more frames or the
% player is closed (if a player object were used).
while hasFrame(ousterReader)
    % Read a frame from the ousterReader, obtaining the point cloud and
    % associated attributes.
    [ptCloud, attributes] = readFrame(ousterReader);

    % Store the point cloud and near-infrared data in cell arrays.
    storage{i} = ptCloud;
    nearIr{i} = attributes.NearInfrared;

    % Increment the counter.
    i = i + 1;
end

%% 3. Point Cloud Processing and Visualization

% Define the region of interest (ROI) as [xmin xmax ymin ymax zmin zmax].
roi = [-.5 -.2 -0.15 0.15 -0.15 0.15];

% Define the minimum distance between points for clustering (in meters).
minDistance = 0.009;

% Define the minimum number of points required for a cluster to be considered valid.
minPoints = 1000;

% Create a figure for displaying the point clouds.
figure;

% Initialize the previous centroid to an empty array.
prevCentroid = [];

%Takeoff Drone
%takeoff(droneObj);

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

    % Get the cluster labels for the valid points.
    labelColorIndex = labels(idxValidPoints);

    % Select the valid points to create a segmented point cloud.
    segmentedPtCloud = select(ptCloudB, idxValidPoints);

    % Set the colormap for visualizing the clusters.
    colormap(hsv(numClusters));

    % Check if the segmented point cloud is empty.
    if isempty(segmentedPtCloud.Location)
        disp('The point cloud is empty.');

        % Set direction to zero since the point cloud is empty
        direction = [0, 0, 0];

        % % Move the drone (commented out).
        % if ~isempty(droneObj)
        %     move(droneObj, direction);
        % end

        % Reset the previous centroid.
        prevCentroid = [];

        % Skip to the next iteration.
        continue;
    end

    % Display the segmented point cloud with color-coded clusters.
    pcshow(segmentedPtCloud.Location, labelColorIndex, "MarkerSize", 30);
    set(gca, 'Color', [1 1 1]); % Set background color to white
    title('Point Cloud Clusters');

    % Set the viewing angle.
    v = [-1, -0.5, -0.5];
    [caz, cel] = view(v);

    % Calculate the centroid of the current segmented point cloud.
    currentCentroid = mean(segmentedPtCloud.Location, 1);

    % If there is a previous centroid, calculate the direction vector.
    if ~isempty(prevCentroid)
        %strech the direction vector by factor 10 from 0.3 m to 3 m max 
        direction = 10*(currentCentroid - prevCentroid);

        fprintf('Direction vector from previous to current centroid: [%f, %f, %f]\n', direction);

        % % Move the drone (commented out).
        % if ~isempty(droneObj)
        %     move(droneObj, direction);
        % end

        % Display the direction vector as a quiver plot.
        hold on;
        quiver3(prevCentroid(1), prevCentroid(2), prevCentroid(3), direction(1), direction(2), direction(3), 0, 'k', 'LineWidth', 2, 'MaxHeadSize', 2);
        hold off;
    else
        % Set direction to zero if there is no previous centroid.
        direction = [0, 0, 0];

        % % Move the drone (commented out).
        % if ~isempty(droneObj)
        %     move(droneObj, direction);
        % end
    end

    % Update the previous centroid.
    prevCentroid = currentCentroid;

    % Pause for a short duration to allow visualization.
    pause(0.1);
end

%% 4. Drone Landing (Commented Out)

% Set the direction to zero before landing.
direction = [0, 0, 0];

% Land the drone
% land(droneObj);
