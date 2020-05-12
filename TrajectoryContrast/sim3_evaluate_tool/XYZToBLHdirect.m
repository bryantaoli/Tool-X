function [ BLH ] = XYZToBLHdirect( XYZ )
%XYZTOBLHDIRECT 参考文献：曾启雄. 空间直角坐标直接解算大地坐标的闭合公式[J]. 测绘学报, 1981(2):3-15.
%   此函数用直接法解出空间直角坐标转到大地坐标XYZ输入为列向量  %代码作者：李涛,单位：上海交通大学
    a=6378137.0000;
    e=0.081819190928906327;
    X=XYZ(1,1);
    Y=XYZ(2,1);
    Z=XYZ(3,1);
    D0=sqrt(1-e*e)*Z/(sqrt(X*X+Y*Y+(1-e*e)*Z*Z));
    C0=sqrt(X*X+Y*Y)/(sqrt(X*X+Y*Y+(1-e*e)*Z*Z));
    A0=a*e*e/(sqrt(X*X+Y*Y+(1-e*e)*Z*Z));
    K1=54*A0*C0*D0*sqrt((1-A0*A0)*(1-A0*A0)*(1-A0*A0)/27+A0*A0*C0*C0*D0*D0);
    K=(1-A0*A0)*(1-A0*A0)*(1-A0*A0)+54*A0*A0*C0*C0*D0*D0;
    one_three=1/3;
    p=1/3*((K+K1)^one_three+(K-K1)^one_three+2*A0*A0+C0*C0-2*D0*D0);
    E=C0-sqrt(p)+sqrt(2*A0*A0+C0*C0-2*D0*D0-p+2*C0*(A0*A0+D0*D0)/(sqrt(p)));
    if(Z==0)
        BLH(1,1)=0;
    else if(X==0&&Y==0)
        BLH(1,1)=pi/2;    
         else
        BLH(1,1)=atan2(sqrt(4*A0*A0-E*E),(E*sqrt(1-e*e))); 
         end
    end
    BLH(2,1)=atan2(Y,X);
    N = a / (sqrt(1 - e*e*sin(BLH(1,1))*sin(BLH(1,1))));
    if(Z~=0)
        BLH(3,1)=Z/sin(BLH(1,1))-N*(1-e*e);
    else
        BLH(3,1)=10;%当Z是0的时候，任意H都行,这边随便给了个高程
    end
end

