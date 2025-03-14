clear; clc; close all;
%% 3. Point Cloud Processing and Visualization
% Define the initial region of interest (ROI) for pattern search.
roi = [-.5 -.2 -0.15 0.15 -0.15 0.15];
%Define display region of interest
displayRoi = [roi(1)*2 roi(2) roi(3)*4 roi(4)*4 roi(5)*4 roi(6)*4];
% Define the minimum distance between points for clustering (in meters).
minDistance = 0.009;
% Define the minimum number of points required for a cluster to be considered valid.
minPoints = 1000;
% Create a figure for displaying the point clouds.
figure;
% Initialize the previous centroid to an empty array.
prevCentroid = [];
firstColor = [0/255 81/255 149/255]; % RGB: 0/81/149 (Divided by 255 for MATLAB's range)
handColor = [1/255 160/255 233/255]; % RGB: 1/160/233
vectorColor = [234/255 113/255 37/255]; % RGB: 234/113/37 (Divided by 255 for MATLAB's range)

% Get the corners of the ROI cube for plotting
roi_corners = [
    roi(1), roi(3), roi(5); % xmin, ymin, zmin
    roi(2), roi(3), roi(5); % xmax, ymin, zmin
    roi(2), roi(4), roi(5); % xmax, ymax, zmin
    roi(1), roi(4), roi(5); % xmin, ymax, zmin
    roi(1), roi(3), roi(6); % xmin, ymin, zmax
    roi(2), roi(3), roi(6); % xmax, ymin, zmax
    roi(2), roi(4), roi(6); % xmax, ymax, zmax
    roi(1), roi(4), roi(6)  % xmin, ymax, zmax
];

% Define the edges of the cube for plotting
cube_edges = [
    1 2; 2 3; 3 4; 4 1;  % Bottom face
    5 6; 6 7; 7 8; 8 5;  % Top face
    1 5; 2 6; 3 7; 4 8   % Vertical edges
];
pause(1);
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

%% Loop
% Loop through the stored point clouds, starting from the 1st frame.
for i = 1:length(storage)

    % Find the indices of points within the defined ROI.
    indices = findPointsInROI(storage{1, i}, roi);
    % Select the points within the ROI to create a new point cloud.
    ptCloudB = select(storage{1, i}, indices);

    % Find the indices of points within the display ROI.
    displayIndices = findPointsInROI(storage{1, i}, displayRoi);
    % Select the points within the display ROI.
    displayPtCloud = select(storage{1, i}, displayIndices);

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

    % Display everything
    pcshow(displayPtCloud.Location, [0.5 0.5 0.5], "MarkerSize", 30);

    hold on;

    % Plot the ROI cube
    for edge = 1:size(cube_edges, 1)
        p1 = roi_corners(cube_edges(edge, 1), :);
        p2 = roi_corners(cube_edges(edge, 2), :);
        plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], 'Color', firstColor, 'LineWidth', 2);  % ROI Cube Lines
    end

    % Display the vector
    if ~isempty(prevCentroid)
        direction = currentCentroid - prevCentroid;
        direction_drone = 10*direction;
        if direction_drone(1)>0.2 || direction_drone(2)>0.2 || direction_drone(3)>0.2
            fprintf('Move drone to: [%f, %f, %f]\n', direction_drone);
            quiver3(prevCentroid(1), prevCentroid(2), prevCentroid(3), direction_drone(1), direction_drone(2), direction_drone(3), 0, 'Color', vectorColor, 'LineWidth', 2, 'MaxHeadSize', 2);
        else
            %fprintf('Hand didnt move enough: [%f, %f, %f]\n', direction_drone);
            continue;
        end
    end

    %Plot again the segment for good vector visualization
    pcshow(segmentedPtCloud.Location, handColor, "MarkerSize", 30);
    hold off;

    % Update the previous centroid.
    prevCentroid = currentCentroid;

    % Set axes limits to display ROI
    xlim([displayRoi(1), displayRoi(2)]);
    ylim([displayRoi(3), displayRoi(4)]);
    zlim([displayRoi(5), displayRoi(6)]);

    set(gca, 'Color', [1 1 1]); % Set background color to white
    title('Segmented Point Cloud, Centroid, ROI and Displacement Vector');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');

    % Set the viewing angle.
    v = [-1, -0.5, -0.5];
    [caz, cel] = view(v);

    % Pause for a short duration to allow visualization.
    pause(0.1);
end
disp('Landing Requested');