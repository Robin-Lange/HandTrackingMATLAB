go into the json and change "build_rev":"v3.0.0"
clear
clc
close all
ousterReader = ousterFileReader("OS-0-128_122313000402_2048x10_20250305_194700.pcap","OS-0-128_122313000402_2048x10_20250305_194700.json");
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
roi = [-1.5 -.5 -0.8 -0.4 0 0.6];
minDistance = 0.02;
minPoints = 100;
% Create a figure
figure
for i = 40:length(storage)
    indices = findPointsInROI(storage{1, i},roi);
    ptCloudB = select(storage{1, i},indices);
    [labels,numClusters] = pcsegdist(ptCloudB,minDistance, NumClusterPoints=minPoints);
    idxValidPoints = find(labels);
    labelColorIndex = labels(idxValidPoints);
    segmentedPtCloud = select(ptCloudB,idxValidPoints);
    colormap(hsv(numClusters))
    pcshow(segmentedPtCloud.Location,labelColorIndex,"MarkerSize",10)
    set(gca, 'Color', [1 1 1]);
    title('Point Cloud Clusters')
    pause(1);
end