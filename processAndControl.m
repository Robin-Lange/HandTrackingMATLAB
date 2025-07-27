function [xdir, ydir, zdir, yaw] = processAndControl(ptCloud, droneObj, p_ref, denoise)
    % PROCESSANDCONTROL Processes the point cloud and sends control commands to the drone.
    %
    %   [xdir, ydir, zdir, yaw] = PROCESSANDCONTROL(ptCloud, droneObj, p_ref, denoise)
    %   processes the point cloud, calculates the control inputs, and sends
    %   commands to the drone.
    %
    %   Inputs:
    %       ptCloud  - The input point cloud.
    %       droneObj - The drone object.
    %       p_ref    - The reference position.
    %       denoise  - A boolean to enable/disable denoising.
    %
    %   Outputs:
    %       xdir     - The x-direction control input.
    %       ydir     - The y-direction control input.
    %       zdir     - The z-direction control input.
    %       yaw      - The yaw control input.

    %% 1. Manipulate Point Cloud
    % Define the region of interest (ROI) for pattern search.
    roi = [0.3 0.6 -.15 .15 -.15 .15];
    segmented_ptCloud = pointextractor(ptCloud, roi);

    if isempty(segmented_ptCloud.Location)
        disp('No hand detected');
        xdir = 0; ydir = 0; zdir = 0; yaw = 0;
        % Skip to the next iteration.
        return;
    end

    if denoise
        clean_ptCloud = pcdenoise(segmented_ptCloud);
    else
        clean_ptCloud = segmented_ptCloud;
    end

    %% 2. Get the control inputs
    % Fit the plane using the affine_fit function
    [T, p, xdir, ydir, yaw] = affine_fit(clean_ptCloud.Location);
    zdir = p(1) - p_ref;

    %% 3. Control the drone
    if abs(yaw) > 0.1
        fprintf('Turn drone about: %f\n', yaw);
        % turn(droneObj, yaw);
    end

    if abs(xdir) > 0.2 || abs(ydir) > 0.2 || abs(zdir) > 0.2
        fprintf('Move drone to: [%f, %f, %f]\n', [xdir, ydir, zdir]);
        % move(droneObj, [xdir, ydir, zdir], 'Speed', 0.1);
    else
        fprintf('Hand didnt move enough: [%f, %f, %f]\n', [xdir, ydir, zdir]);
    end
end
