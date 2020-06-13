function [q_all] = euler2quatdegrall(eulerdegall)
%EULER2QUATDEGRALL Summary of this function goes here
%   Detailed explanation goes here
    q_all = zeros(size(eulerdegall,1), 4);
    for i = 1 : size(eulerdegall,1)
        roll = eulerdegall(i,1) / 180 * pi;
        pitch = eulerdegall(i,2) / 180 * pi;
        yaw = eulerdegall(i,3) / 180 * pi;
        q = angle2quat(roll,pitch,yaw,'ZYX');
        q_all(i:i , 1:4) = q;
        q_all(i:i , 1:4);
    end
end

