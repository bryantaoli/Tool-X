function [  ] = matTotxt( X ,Y)
%MATTOTXT Summary of this function goes here
%   Detailed explanation goes here
[r,c]=size(X);
fid=fopen(Y,'w');
for i=1:r
    for j=1:c
        fprintf(fid,'%20.10f\t',X(i,j));
        if(rem(j,c)==0)
            fprintf(fid,'\r\n');
        end
    end
end
end

