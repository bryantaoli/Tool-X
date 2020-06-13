function [ BLH ] = XYZToBLH( XYZ )
e=0.081819190928906327;
a=6378137;
DeltaZ = e*e*XYZ(3,1);
d0 = 1.0;
d1 = 0.0;
for i = 0:1:200
    if (abs(d1 - d0) < 0.000000000001)
        break;
    else
        d1 = d0;
        d0 = asin((XYZ(3,1) + DeltaZ) / sqrt(XYZ(1,1)^2 + XYZ(2,1)^2 + (XYZ(3,1) + DeltaZ)^2));
        BLH(1,1) = d0;
        N = a / sqrt(1 - e*e*sin(BLH(1,1))*sin(BLH(1,1)));
        DeltaZ = N * sin(BLH(1,1)) * e*e;
    end
end
BLH(1,1) = atan2((XYZ(3,1) + DeltaZ), sqrt(XYZ(1,1)^2 + XYZ(2,1)^2));
BLH(2,1) = atan2(XYZ(2,1), XYZ(1,1));
BLH(3,1) = sqrt(XYZ(1,1) * XYZ(1,1) + XYZ(2,1) * XYZ(2,1) + (XYZ(3,1) + DeltaZ)*(XYZ(3,1) + DeltaZ)) - N;

end

