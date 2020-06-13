clear; clc;
load('reference.mat');
t_ned_q = zeros(size(reference,1),8);
t_ned_q(:,1) = reference(:,1);
t_ned_q(:,2:4) = BLHToNEDdegall(reference(:,2:4));

%plot3(ned(:,1),ned(:,2),ned(:,3))
q = euler2quatdegrall(reference(:,5:7));

t_ned_q(:,5:8) = q;
matTotum(t_ned_q,'ECEF.tum');