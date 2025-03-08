droneObj = ryze();
%i=0;
takeoff(droneObj);
pause(5);
%https://www.mathworks.com/help/matlab/supportpkg/move.html#mw_999f8f54-c63d-4aa0-9003-ad7b8c3ae355
move(droneObj,[1.2 1.2 0],'Speed', 0.5);
pause(5);
land(droneObj);

%abort(droneObj);

