function [ R, t ] = se3DataAlignment( X, Y )
%SO3DATAALIGNMENT 此处显示有关此函数的摘要 找一个R,t使得Y=RX+t
%   此处显示详细说明ssss
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

