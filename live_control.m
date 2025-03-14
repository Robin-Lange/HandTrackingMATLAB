clear; clc; close all;
warning('off','all');

%% 3. Point Cloud Processing and Visualization
% Define the initial region of interest (ROI) for pattern search.
roi = [0.2 0.5 -.15 .15 0 0.30];
% Define the minimum distance between points for clustering (in meters).
minDistance = 0.009;
% Define the minimum number of points required for a cluster to be considered valid.
minPoints = 1000;
% Initialize the previous centroid to an empty array.
prevCentroid = [];
%% 1. Sensor Initialization and Viewfinder
ousterObj = ousterlidar("OS0-128","OS-0-128_122313000402.json");
preview(ousterObj,[0.2 0.5],[-.15 .15], [0 .30]);
pause(3)
closePreview(ousterObj);
%pre_pointcloud=[];
start(ousterObj);
pre_pointcloud=read(ousterObj,1, "latest");

%connect to the drone
%droneObj = ryze();
%takeoff(droneObj);
%% Loop
% Loop through the stored point clouds, starting from the 1st frame.

for i = 1:1000
    tic
    [ptCloud, pcAttributes] = read(ousterObj,1, "latest");
    
    if ptCloud ~= pre_pointcloud
        % Find the indices of points within the defined ROI.
        indices = findPointsInROI(ptCloud, roi);
        % Select the points within the ROI to create a new point cloud.
        ptCloudB = select(ptCloud, indices);
    
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
                %move(droneObj,direction_drone);
            else
                %fprintf('Hand didnt move enough: [%f, %f, %f]\n', direction_drone);
                Toc(i) = toc;
                continue;
            end
        end
        % Update the previous centroid.
        prevCentroid = currentCentroid;
        pre_pointcloud=ptCloud;
    end
    Toc(i) = toc;
end
disp('Landing Requested');
stop(ousterObj);
%land(droneObj);