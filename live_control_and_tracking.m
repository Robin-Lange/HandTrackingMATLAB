clear; clc; close all;
warning('off','all');

%% 1. Point Cloud Parameters
% Define the initial region of interest (ROI) for pattern search.
roi = [0.3 0.6 -.15 .15 -.15 .15];
figure;

%% 2. Sensor Initialization and Viewfinder
ousterObj = ousterlidar("OS0-128","OS-0-128_122313000402.json");
horizontalResolution = 2048;
params = lidarParameters('OS0-128', horizontalResolution, SweepStartAngle=135);
preview(ousterObj,[0.3 0.6],[-.15 .15], [0 .30]);
pause(3)
closePreview(ousterObj);
start(ousterObj);
pause(3);

%connect to the drone
droneObj = ryze();
cameraObj = camera(droneObj);
takeoff(droneObj);
frame = snapshot(cameraObj);
preview(cameraObj);
%% 3.Loop
for j = 1:1000
    %% 4. Manipulate Point Cloud
    % Find the indices of points within the ROI.
    segmented_ptCloud = pointextractor(read(ousterObj, 1, "latest"),roi);
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
    pause(0.5);
end
disp('Landing Requested');
land(droneObj);
closePreview(cameraObj);
stop(ousterObj);