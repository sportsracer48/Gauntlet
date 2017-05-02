function [R, T] = getScanTransform(tf, timestamp)
    try
        odom_pose = getTransform(tf, 'odom', 'base_laser_link', timestamp, 'Timeout', 0.5);
    catch ME
        disp('unable to get transform.... Ignore the returned R and T');
        R = [];
        T = [];
        return;
    end
    trans = odom_pose.Transform.Translation;
    quat = odom_pose.Transform.Rotation;
    rot = quat2eul([quat.W quat.X quat.Y quat.Z]);
    % the laser frame is rotated by 180 degrees to adhere to various ROS
    % conventions... Here, we undo it.
    yaw = rot(1) + pi;
    R = [cos(yaw) sin(-yaw) 0;...
         sin(yaw) cos(yaw) 0;...
         0 0 1];
    T = [1 0 trans.X;
         0 1 trans.Y;
         0 0 1];
end