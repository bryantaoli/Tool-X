function [  ] = matTotum( matname ,textname)
%MATTOTXT Summary of this function goes here
%   Detailed explanation goes here
[r,c]=size(matname);
fid=fopen(textname,'w');
for i=1:r
    for j=1:c
        if(rem(j,c)~=0)
            fprintf(fid,'%.5f ',matname(i,j));
        end
        
        if(rem(j,c)==0)
            fprintf(fid,'%.5f',matname(i,j));
            fprintf(fid,'\n');
        end
    end
end
fclose(fid)
end

