clear
clc
close all

% Initialize the ousterFileReader with the .pcap and .json files
ousterReader = ousterFileReader("OS-0-128_122313000402_2048x10_20250306_183057.pcap", "OS-0-128_122313000402_2048x10_20250306_183057.json");
% Create a figure for visualization
figure

% Iterate through each frame and visualize the point cloud data
while hasFrame(ousterReader)
    [ptCloud, ~] = readFrame(ousterReader);
    pcshow(ptCloud)
    title('LiDAR Point Cloud Frame')
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    pause(0.1) % Adjust the pause duration as needed to control playback speed
end