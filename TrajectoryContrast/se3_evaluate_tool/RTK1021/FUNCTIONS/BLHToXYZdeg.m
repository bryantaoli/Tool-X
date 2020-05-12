function [ XYZ ] = BLHToXYZdeg( BLH_deg )
%BLHTOXYZ Summary of this function goes here
%   Detailed explanation goes here
    XYZ=zeros(size(BLH_deg,1),3);
    for i=1:size(BLH_deg,1)
        BLH=[BLH_deg(i,1)/180*pi;BLH_deg(i,2)/180*pi;BLH_deg(i,3)];
        a=6378137.0000;
        e=0.081819190928906327;
        N = a / (sqrt(1 - e*e*sin(BLH(1,1))*sin(BLH(1,1))));
        XYZ(i,1) = (N + BLH(3,1))*cos(BLH(1,1))*cos(BLH(2,1));
        XYZ(i,2) = (N + BLH(3,1))*cos(BLH(1,1))*sin(BLH(2,1));
        XYZ(i,3) = (N*(1 - e*e) + BLH(3,1))*sin(BLH(1,1));  
    end
end

