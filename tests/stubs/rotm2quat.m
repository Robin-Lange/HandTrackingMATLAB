function quat = rotm2quat(rotm)
    tr = trace(rotm);
    if tr > 0
        s = sqrt(tr + 1.0) * 2;
        qw = 0.25 * s;
        qx = (rotm(3, 2) - rotm(2, 3)) / s;
        qy = (rotm(1, 3) - rotm(3, 1)) / s;
        qz = (rotm(2, 1) - rotm(1, 2)) / s;
    elseif (rotm(1, 1) > rotm(2, 2)) && (rotm(1, 1) > rotm(3, 3))
        s = sqrt(1.0 + rotm(1, 1) - rotm(2, 2) - rotm(3, 3)) * 2;
        qw = (rotm(3, 2) - rotm(2, 3)) / s;
        qx = 0.25 * s;
        qy = (rotm(1, 2) + rotm(2, 1)) / s;
        qz = (rotm(1, 3) + rotm(3, 1)) / s;
    elseif rotm(2, 2) > rotm(3, 3)
        s = sqrt(1.0 + rotm(2, 2) - rotm(1, 1) - rotm(3, 3)) * 2;
        qw = (rotm(1, 3) - rotm(3, 1)) / s;
        qx = (rotm(1, 2) + rotm(2, 1)) / s;
        qy = 0.25 * s;
        qz = (rotm(2, 3) + rotm(3, 2)) / s;
    else
        s = sqrt(1.0 + rotm(3, 3) - rotm(1, 1) - rotm(2, 2)) * 2;
        qw = (rotm(2, 1) - rotm(1, 2)) / s;
        qx = (rotm(1, 3) + rotm(3, 1)) / s;
        qy = (rotm(2, 3) + rotm(3, 2)) / s;
        qz = 0.25 * s;
    end

    quat = [qw, qx, qy, qz];
    quat = quat ./ norm(quat);
end
