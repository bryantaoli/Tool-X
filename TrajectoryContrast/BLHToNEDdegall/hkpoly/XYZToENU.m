function [ ENU ] = XYZToENU( XYZA1,XYZA2 )
    BLHA2=XYZToBLHdirect(XYZA2);
    M=zeros(3,3);
    M(1,1)=-sin(BLHA2(2));
    M(1,2)=cos(BLHA2(2));
    M(1,3)=0;
    M(2,1)=-sin(BLHA2(1))*cos(BLHA2(2));
    M(2,2)=-sin(BLHA2(1))*sin(BLHA2(2));
    M(2,3)=cos(BLHA2(1));
    M(3,1)=cos(BLHA2(1))*cos(BLHA2(2));
    M(3,2)=cos(BLHA2(1))*sin(BLHA2(2));
    M(3,3)=sin(BLHA2(1));
    delta_A1_A2=zeros(3,1);
    delta_A1_A2(1)=XYZA1(1)-XYZA2(1);
    delta_A1_A2(2)=XYZA1(2)-XYZA2(2);
    delta_A1_A2(3)=XYZA1(3)-XYZA2(3);
    ENU=M*delta_A1_A2;
end

