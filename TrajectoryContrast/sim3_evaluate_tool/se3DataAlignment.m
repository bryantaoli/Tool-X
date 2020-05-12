%% align data by se3 method
%   Author:         Tao Li from ShangHaiJiaoTong University
%   Announcement:   Not debug free. Feel free to do any modifications and
%                   use it at will
%-Inputs:
% @X: 3*n data to be aligned with @Y
% @Y: 3*n data to be aligned with @X
%-Outputs:
% @R: rotation
% @t: transition

function [ R, t ] = se3DataAlignment( X, Y )
%找一个R,t使得Y=RX+t

n = size(X,2);
mu_X = sum(X,2)/n;  % mean of X
mu_Y = sum(Y,2)/n;  % mean of Y
x=X-repmat(mu_X,1,n);
y=Y-repmat(mu_Y,1,n);
W=zeros(3,3);
    for i=1:size(X,2)
        %W=W+x(:,i)*y(:,i)';这种写法是不对的
        W=W+y(:,i)*x(:,i)';
    end
[u,d,v]=svd(W);
R=u*v';
%t=mu_X-R*mu_Y;这种写法是不对的
t=mu_Y-R*mu_X;
end
