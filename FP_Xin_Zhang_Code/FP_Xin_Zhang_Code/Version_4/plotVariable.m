function plotVariable()
%  figure of the trajectory, velocity and acceleration
global Av1
global Av2
global Xv1
global Xv2
global Vv1
global Vv2
t = 1:size(Av1,2);
% acceleration
figure(5)
Av2=Av2';
subplot(3,2,1)

plot(t,Av1(1,:));
hold on
plot(t,Av2(1,t));
hold off
title('X - Acceleration');
subplot(3,2,2)
plot(t,Av1(2,:));
hold on 
plot(t,Av2(2,t));
hold off
title('Y- Acceleration');

% speed
subplot(3,2,3)
plot(t,Vv1(:,1));
hold on 
plot(t,Vv2(:,1));
hold off
title('X- Speed');
subplot(3,2,4)
plot(t,Vv1(:,2));
hold on 
plot(t,Vv2(:,2));
hold off
title('Y- Speed');

% road
subplot(3,2,5)
plot(t,Xv1(:,1));
hold on 
plot(t,Xv2(:,1));
hold off
title('X- Road');
subplot(3,2,6)
plot(t,Xv1(:,2))
hold on 
plot(t,Xv2(:,2))
hold off
title('Y- Road');
end