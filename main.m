clear
clc
close all

ousterReader = ousterFileReader("OS-0-128_122313000402_2048x10_20250306_183057.pcap","OS-0-128_122313000402_2048x10_20250306_183057.json");
ousterReader.CurrentTime = ousterReader.StartTime + seconds(0.3);
%connect to the drone
%droneObj = ryze();

i = 1;
while(hasFrame(ousterReader))% && player.isOpen())
    [ptCloud, attributes] = readFrame(ousterReader);
    % view(player,ptCloud(1,1));
    storage{i} = ptCloud;
    nearIr{i} = attributes.NearInfrared;
    i = i+1;
end
%%
roi = [-.5 -.2 -0.15 0.15 -0.15 0.15];
minDistance = 0.009;
minPoints = 1000;
% Create a figure
figure
prevCentroid = []; % Initialize previous centroid
for i = 40:length(storage)
    indices = findPointsInROI(storage{1, i},roi);
    ptCloudB = select(storage{1, i},indices);
    [labels, numClusters] = pcsegdist(ptCloudB, minDistance, NumClusterPoints=minPoints);
    idxValidPoints = find(labels);
    labelColorIndex = labels(idxValidPoints);
    segmentedPtCloud = select(ptCloudB, idxValidPoints);
    colormap(hsv(numClusters));
    if isempty(segmentedPtCloud.Location)
        disp('The point cloud is empty.');
        % Set direction to zero since the point cloud is empty
        direction = [0, 0, 0];
        %move(droneObj,direction);
        prevCentroid = []; % Reset previous centroid
        continue;
    elseif i == 1
        % Execute this command only once during the first iteration
        disp('This command is executed only once during the first run.');
        %takeoff(droneObj);
    end
    pcshow(segmentedPtCloud.Location, labelColorIndex, "MarkerSize", 30)
    set(gca, 'Color', [1 1 1]);
    title('Point Cloud Clusters');
    v = [-1,-0.5,-0.5];
    [caz, cel] = view(v);
    
    % Calculate the centroid of the current segmented point cloud
    currentCentroid = mean(segmentedPtCloud.Location, 1);
    
    % If there is a previous centroid, calculate the direction vector
    if ~isempty(prevCentroid)
        direction = currentCentroid - prevCentroid;
        
        % Normalize the direction vector to a unit vector
        magnitude = norm(direction);
        if magnitude ~= 0
            direction = direction / magnitude;
        end
        
        fprintf('Normalized direction vector from previous to current centroid: [%f, %f, %f]\n', direction);
        %move(droneObj,direction);
        
        % Display the vector
        hold on;
        quiver3(prevCentroid(1), prevCentroid(2), prevCentroid(3), direction(1), direction(2), direction(3), 0, 'k', 'LineWidth', 2, 'MaxHeadSize', 2);
        hold off;
    else
        % Set direction to zero if there is no previous centroid
        direction = [0, 0, 0];
        %move(droneObj,direction);
    end
    
    % Update the previous centroid
    prevCentroid = currentCentroid;
    
    pause(0.1);
end
direction = [0, 0, 0];
%land(droneObj);