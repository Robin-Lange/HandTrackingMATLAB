%go into the json and change "build_rev":"v3.0.0"
clear
clc
close all

ousterReader = ousterFileReader("OS-0-128_122313000402_2048x10_20250306_183057.pcap","OS-0-128_122313000402_2048x10_20250306_183057.json");
ousterReader.CurrentTime = ousterReader.StartTime + seconds(0.3);
i = 1;
while(hasFrame(ousterReader) )%&& player.isOpen())
    [ptCloud, attributes] = readFrame(ousterReader);
    %view(player,ptCloud(1,1));
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
for i = 1:length(storage)
    indices = findPointsInROI(storage{1, i},roi);
    ptCloudB = select(storage{1, i},indices);
    [labels,numClusters] = pcsegdist(ptCloudB,minDistance, NumClusterPoints=minPoints);
    idxValidPoints = find(labels);
    labelColorIndex = labels(idxValidPoints);
    segmentedPtCloud = select(ptCloudB,idxValidPoints);
    colormap(hsv(numClusters));
    if isempty(segmentedPtCloud.Location)
        disp('The point cloud is empty.');
        continue;
    end
    pcshow(segmentedPtCloud.Location,labelColorIndex,"MarkerSize",30)
    set(gca, 'Color', [1 1 1]);
    title('Point Cloud Clusters');
    v = [-1,-0.5,-0.5];
    [caz,cel] = view(v);
    pause(0.1);
end