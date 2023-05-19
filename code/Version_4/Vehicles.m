function [u,v,x]=Vehicles(v0,x0,t)
% generate a trajectory with constant speed for target vehicle
% v0, x0: initial velocity and initial position of target vehicle
% t: discrete time 
len = size(t,2);
% x
u(:,1) = zeros(len,1); % moving at constant speed
v(:,1) = v0(1,1) + u(:,1).*t';
x(:,1) = x0(1,1) + v(:,1).*t' + 1/2*u(:,1).*t'.^2;
% y
u(:,2) = zeros(len,1);
v(:,2) = v0(1,2) + u(:,2).*t';
x(:,2) = x0(1,2) + v(:,2).*t' + 1/2*u(:,2).*t'.^2;
end

