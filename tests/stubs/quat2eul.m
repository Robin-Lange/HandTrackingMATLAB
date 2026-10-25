function eul = quat2eul(quat)
    quat = quat ./ norm(quat);
    qw = quat(1);
    qx = quat(2);
    qy = quat(3);
    qz = quat(4);

    yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy^2 + qz^2));
    pitch = asin(max(-1, min(1, 2 * (qw * qy - qz * qx))));
    roll = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx^2 + qy^2));

    eul = [yaw, pitch, roll];
end
