clear; clc; close all;

%% 2. Point Cloud Processing Parameters
roi = [0.2 0.5 -.15 .15 -.15 .15];
displayRoi = [roi(1) roi(2)*2 roi(3)*4 roi(4)*4 roi(5)*4 roi(6)*4];
% Define the minimum distance between points for clustering (in meters).
minDistance = 0.2;
% Define the minimum number of points required for a cluster to be considered valid.
minPoints = 100;
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
% Scale factor for the direction vector (adjust as needed)
scale_factor = 0.3;

%% 1. Sensor Initialization and Viewfinder
ousterObj = ousterlidar("OS0-128","OS-0-128_122313000402.json");
preview(ousterObj,[0.2 0.5],[-.15 .15], [-.15 .15]);
pause(5)
closePreview(ousterObj);

%% Loop
for i=1:1000
    ptCloud = read(ousterObj);

    % Find the indices of points within the defined ROI.
    indices = findPointsInROI(ptCloud, roi);
    % Select the points within the ROI to create a new point cloud.
    ptCloudB = select(ptCloud, indices);

    % Find the indices of points within the display ROI.
    displayIndices = findPointsInROI(ptCloud, displayRoi);
    % Select the points within the display ROI.
    displayPtCloud = select(ptCloud, displayIndices);

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
        % Normalize the direction vector to a unit vector
        magnitude = norm(direction);
        if magnitude ~= 0
            direction = direction / magnitude;
        end
        fprintf('Normalized direction vector from previous to current centroid: [%f, %f, %f]\n', direction);
        scaled_direction = direction * scale_factor;  %Scaling
        quiver3(prevCentroid(1), prevCentroid(2), prevCentroid(3), scaled_direction(1), scaled_direction(2), scaled_direction(3), 0, 'Color', vectorColor, 'LineWidth', 2, 'MaxHeadSize', 2);
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