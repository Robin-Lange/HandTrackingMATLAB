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


figure
for i = 40:length(storage)

    indices = findPointsInROI(storage{1, i},roi);
    ld = load('drivingLidarPoints.mat');
    maxDistance = 0.05;
    referenceVector = [0 0 1];
    [~,inliers,outliers] = pcfitplane(ld.ptCloud,maxDistance,referenceVector);
    ptCloudWithoutGround = select(ld.ptCloud,outliers);
    minDistance = 2;
    minPoints = 10;
    [labels,numClusters] = pcsegdist(ptCloudWithoutGround,minDistance,Method="exhaustive",...
    NumClusterPoints=minPoints);
    idxValidPoints = find(labels);
    labelColorIndex = labels(idxValidPoints);
    segmentedPtCloud = select(ptCloudWithoutGround,idxValidPoints);
    figure
    colormap(hsv(numClusters))
    pcshow(segmentedPtCloud.Location,labelColorIndex)
    title('Point Cloud Clusters')

    ptCloudB = select(storage{1, i},indices);
    pcshow(ptCloudB)
    pause(1);
end