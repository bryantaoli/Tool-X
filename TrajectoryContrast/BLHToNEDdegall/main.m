load('blhdeg.mat');
ned = BLHToNEDdegall(blhdeg);
plot3(ned(:,1),ned(:,2),ned(:,3))