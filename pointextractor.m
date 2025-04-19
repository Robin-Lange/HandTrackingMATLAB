function [segmented_ptCloud] = pointextractor(ptCloud,roi)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    minDistance = 0.009;
    minPoints = 1000;
    indices = findPointsInROI(ptCloud, roi);
    % Select the points within the ROI to create a new point cloud.
    select_ptCloud = select(ptCloud, indices);
    % Segment the point cloud into clusters based on distance and minimum point count.
    [labels, numClusters] = pcsegdist(select_ptCloud, minDistance, NumClusterPoints=minPoints);
    % Find the indices of valid points (points that belong to a cluster).
    idxValidPoints = find(labels);
    % Select the valid points to create a segmented point cloud.
    segmented_ptCloud = select(select_ptCloud, idxValidPoints);
end

