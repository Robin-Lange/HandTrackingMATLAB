clear; clc; close all;
warning('off','all');

%% 1. Point Cloud Parameters
roi = [-.5 -.2 -0.15 0.15 -0.15 0.15];
figure;

%% 2. Data Loading and Initialization
% Define the file names for the Ouster lidar data and metadata.
pcapFile = "OS-0-128_122313000402_2048x10_20250306_183057.pcap";
jsonFile = "OS-0-128_122313000402_2048x10_20250306_183057.json";
horizontalResolution = 2048;
params = lidarParameters('OS0-128', horizontalResolution, SweepStartAngle=135);
ousterReader = ousterFileReader(pcapFile, jsonFile);
ousterReader.CurrentTime = ousterReader.StartTime + seconds(0.3);
storage = {};      % Cell array to store point clouds
i = 1; % Initialize a counter for storing the data.

while hasFrame(ousterReader)
    % Read a frame from the ousterReader, obtaining the point cloud and
    % associated attributes.
    ptCloudS = readFrame(ousterReader);
    % Store the point cloud and near-infrared data in cell arrays.
    storage{i} = ptCloudS;
    % Increment the counter.
    i = i + 1;
end

%connect to the drone
%%droneObj = ryze();
%cameraObj = camera(droneObj);
%takeoff(droneObj);
%frame = snapshot(cameraObj);
%preview(cameraObj);
%% 3.Loop
for j = 1:length(storage)
    %% 4. Manipulate Point Cloud
    % Find the indices of points within the ROI.
    segmented_ptCloud = pointextractor(storage{1, j},roi);
    if isempty(segmented_ptCloud.Location)
        disp('No hand detected');
        % Skip to the next iteration.
        continue;
    end
    clean_ptCloud = pcdenoise(segmented_ptCloud);

    %% 5. Get the control inputs
    % Fit the plane using the affine_fit function
    [T, p, xdir,ydir,yaw] = affine_fit(clean_ptCloud.Location);
    if j == 1
        p_ref=p(1);
    end
    zdir=p(1)-p_ref;
    if abs(yaw)>0.1
            fprintf('Turn drone about: %f\n', yaw);
            %turn(droneObj,eul(3));
    end
    if abs(xdir)>0.2 | abs(ydir)>0.2 | abs(zdir)>0.2 
            fprintf('Move drone to: [%f, %f, %f]\n', [xdir, ydir, zdir]);
            %move(droneObj,[x_dir, y_dir, z_dir], 'Speed', 0.1);
    else
        fprintf('Hand didnt move enough: [%f, %f, %f]\n', [xdir, ydir, zdir]);
        continue;
    end
    %% 6. Visualize 
    pcshow(clean_ptCloud);
    hold on;
    plot3(p(1),p(2),p(3),'ro','markersize',15,'markerfacecolor','red');
    % Extract x and y ranges from the ROI
    xRange = linspace(roi(1), roi(2), 3); % From -0.5 to -0.2
    yRange = linspace(roi(3), roi(4), 3); % From -0.15 to 0.15
    % Create the meshgrid
    [X, Y] = meshgrid(xRange, yRange);
    %first plane
    surf(X,Y, - (T(1)/T(3)*X+T(2)/T(3)*Y-dot(T(:,1),p)/T(3)),'facecolor','red','facealpha',0.5);
    v_1 = quiver3(p(1),p(2),p(3),T(1)/10,T(2)/10,T(3)/10,'b','linewidth',2);
    v_2 = quiver3(p(1),p(2),p(3),T(4)/10,T(5)/10,T(6)/10,'g','linewidth',2);
    v_3 = quiver3(p(1),p(2),p(3),T(7)/10,T(8)/10,T(9)/10,'r','linewidth',2);
    xlabel('x');
    ylabel('y');
    zlabel('z');
    hold off;
    pause(0.5);
end
disp('Landing Requested');
%land(droneObj);
%closePreview(cameraObj);