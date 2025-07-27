function visualize_results(ptCloud, p, T, roi)
    % VISUALIZE_RESULTS Visualizes the point cloud, the fitted plane, and the normal vector.
    %
    %   VISUALIZE_RESULTS(ptCloud, p, T, roi) displays the point cloud, the
    %   fitted plane, and the normal vector.
    %
    %   Inputs:
    %       ptCloud - The point cloud to visualize.
    %       p       - The point on the plane.
    %       T       - The transformation matrix containing the normal and plane vectors.
    %       roi     - The region of interest.

    pcshow(ptCloud);
    hold on;
    plot3(p(1),p(2),p(3),'ro','markersize',15,'markerfacecolor','red');
    % Extract x and y ranges from the ROI
    xRange = linspace(roi(1), roi(2), 3);
    yRange = linspace(roi(3), roi(4), 3);
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
end
