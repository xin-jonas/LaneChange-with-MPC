%%% ego and Vehicle change Lane
% simulation of the highway scenario
clear all;
close all;
clc;

%% Highway Info
% lane width
global w_lane;
% lane centers;
global cen_up_lane;
global cen_mid_lane;
global cen_low_lane;

%% Car Info
global l; % length of vehicle
global b; % width of vehicle
global Xv1; % position of the first vehicle ( center of a rectangle ) 
global Xv2; % position of the second vehicle
global Vv1; % velocity of the first vehicle
global Vv2; % velocity of the second vehicle

%% MPC Set
global T; % sampling time of MPC
% tracking of the time skips
global iterations;

%% Maneuever set
% Safe distance
global x_safe;
global x_follow;

%% Highway Variable initial
% set lane width
w_lane          = 5.25;
% calculate lane centers (for plot of target vehicle)
cen_up_lane     = 5*w_lane/2;
cen_mid_lane    = 3*w_lane/2;
cen_low_lane    = w_lane/2;

%% MPC variable initial
mpciterations = 1; % number of MPC iterations
tol_opt       = 1e-6;
opt_option    = 0;
iprint        = 5;
type          = 'difference equation';
atol_ode_real = 1e-8;
rtol_ode_real = 1e-8;
atol_ode_sim  = 1e-2;
rtol_ode_sim  = 1e-2;
% MPC iteration set
N             = 25; % prediction horizon
T             = 0.2; % discrete time size

%% Car Infor initial
% Car len and breit
l   = 4.7;
b   = 1.83;
%% initial condition of two vehilces
tmeasure    = 0.0;
x1measure   = [+30.000 +0.000 +10 cen_low_lane]; 
x2measure   = [+25.0 +0.000 +50 cen_mid_lane];
 
% Control Initial
u01         = zeros(2,N);
u02         = zeros(2,N);
% Position
Xv1         =x1measure(end,3:4); %position of the first vehicle
Xv2         =x2measure(end,3:4); %position of the second vehicle
% Velocity
Vv1         =x1measure(end,1:2); %velocity of the first vehicle
Vv2         =x2measure(end,1:2); %velocity of the second vehicle

%% Manuever Plannning Variable
x_safe = 40; % safety distance
x_follow = 10;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Highway Plot
plotHighway();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% analysis of the two vehicles simultaneously
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vehicles in Enviroment
v0 = Vv2;
x0 = Xv2;
t02 = 0:0.2:25;
% target vehicle trajectory generate
[u02,V_v2,X_v2] = Vehicles(v0,x0,t02);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MPC Simulation

t01 = tmeasure;
t02 = tmeasure;

global Av1;
Av1 = [0;0];
global Av2;
Av2 = u02;

tic
for iter=1:125
    iterations(end+1,:) = iter;
    [t01, x1measure, u01] = nmpc(@runningcosts, @terminalcosts1v1, @constraints1, ...
        @terminalconstraints1, @linearconstraints1, @system, ...
        mpciterations, N, T, t01(end,:), x1measure(end,:), u01, ...
        tol_opt, opt_option, ...
        type, atol_ode_real, rtol_ode_real, atol_ode_sim, rtol_ode_sim, ...
        iprint, @printHeader, @printClosedloopData); %for the first vehicle
    
    % update ego vehicle control signal
    Xv1(end+1,:) = x1measure(end,3:4);
    Vv1(end+1,:) = x1measure(end,1:2);
    u01 = [u01(:,2:size(u01,2)) u01(:,size(u01,2))]; 
    Av1(:,end+1) = u01(:,1);
    % update target vehicle state
    Xv2(end+1,:) = X_v2(iter+1,:);
    Vv2(end+1,:) = V_v2(iter+1,:);
    
    plotVehicles();
end
toc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Definition of the NMPC functions for the first vehicles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [c,ceq] = constraints1(t, x, u, kT)

    global b;
    global w_lane;
    global Xv1;
    global Vv1;
    global Xv2;
    global Vv2;

    global x_safe;
    v_x_min = 50/3.6;
    v_x_max = 70;
    c   = [];
    c(1) = x(4)-3*w_lane+b/2;
    c(2) = -x(4)+b/2;
    c(3) = -x(1)+v_x_min;
    c(4) = x(1)-v_x_max;
    
    if (x(4)>w_lane)&&(x(4)<2*w_lane)
        c(5) = -((Xv2(end,1)+Vv2(end,1)*kT)-x(3))^2+x_safe^2;
    else
        c(5) = -x(3);
    end
    c(6) = -x(3);
    ceq = [];
end

function [c,ceq] = terminalconstraints1(t, x, kT)

    global b;
    global w_lane;
    global Xv1;
    global Vv1;
    global Xv2;
    global Vv2;
 
    global x_safe;
        
    v_x_min = 50/3.6;
    v_x_max = 70;
    c   = [];
    c(1) = x(4)-3*w_lane+b/2;
    c(2) = -x(4)+b/2;
    c(3) = -x(1)+v_x_min;
    c(4) = x(1)-v_x_max;
    c(5) = -((((Xv2(end,1)+Vv2(end,1)*kT)-x(3))^2)/(5^2)+(((Xv2(end,2)+Vv2(end,2)*kT)-x(4))^2)/((w_lane/2)^2))+1;
    if (x(4)>w_lane)&&(x(4)<2*w_lane)
        c(5) = -((Xv2(end,1)+Vv2(end,1)*kT)-x(3))^2+x_safe^2;
    else
        c(5) = -x(3);
    end
    c(6) = -x(3);
    ceq = [];
    
end

function [A, b, Aeq, beq, lb, ub] = linearconstraints1(t, x, u)
    a_min = -9; %minimum deceleration 
    a_max = 6;  %maximum acceleration
    a_ymin= -1*0.5; %minimum steering rate
    a_ymax= 1*0.5;  %maximum steering rate
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
    lb  = [a_min a_ymin];
    ub  = [a_max a_ymax];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Definition of the NMPC functions common for the two vehicles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dx = system(t, x, u, T)
    dx = zeros(4,1);
    % double integrator model of the lead vehicle (used for prediction)
    dx(1) = x(1)+u(1)*T;                                               %xd
    dx(2) = x(2)+u(2)*T;                                               %yd
    dx(3) = x(3)+x(1)*T+T^2*u(1)/2;                                    %x                                               
    dx(4) = x(4)+x(2)*T+T^2*u(2)/2;                                    %y  
    %  Needed for TimeDiscrete
    dx=dx.';   
end

function printHeader()
%     fprintf('   k  |    u(1)     u(2)      V(1)     V(2)    X(1)    X(2)    Time\n');
%     fprintf('--------------------------------------------------\n');
end

function printClosedloopData(mpciter, u, x, t_Elapsed)
    
     global iterations;
    
    fprintf(' %3d  | %+11.6f %+11.6f %+6.3f %+6.3f %+6.3f  %+6.3f  %+6.3f', iterations(end,:), ...
            u(1), u(2), x(1), x(2), x(3), x(4) ,t_Elapsed);    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot highway and vehicle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function plotHighway()
% highway and vehicle info
    global w_lane;
    global cen_up_lane;
    global cen_mid_lane;
    global cen_low_lane;
% fig setting
    figure(2);
    set(gcf,'Units','normalized','OuterPosition',[0.02 0.1 0.95 0.25]);
    set(0,'DefaultAxesFontName', 'Times New Roman')
    set(0,'DefaultAxesFontSize', 12)
    set(gcf,'PaperPositionMode','auto')
    set(gcf, 'Color', 'w');
    axis equal;
    fontsize_labels = 12;
    xlim([0 1000]);
    ylim([-0.25 3*w_lane+0.25]);
    set(gcf,'Position',[50 600 3000 200])
    set(gcf, 'Color', 'w');
    set(gca, 'FontSize', 12);
    set(gca,'ytick',[0:10:10]);
    set(gca,'xtick',[0:100:1000]);
    xlabel('$x$(m)','interpreter','latex','FontSize',fontsize_labels);
    ylabel('$y$(m)','interpreter','latex','FontSize',fontsize_labels);
% highway plot
    plot([0 1000], [0 0], '-k', 'LineWidth', 1.5);
    hold on;
    plot([0 1000], [w_lane w_lane], '-k','LineWidth', 1.5);
    plot([0 1000], [2*w_lane 2*w_lane], '-k', 'LineWidth', 1.5);
    plot([0 1000], [3*w_lane 3*w_lane], '-k', 'LineWidth', 1.5);
    plot([0 1000], [cen_up_lane cen_up_lane], '--k', 'LineWidth', 1);
    plot([0 1000], [cen_mid_lane cen_mid_lane], '--k', 'LineWidth', 1);
    plot([0 1000], [cen_low_lane cen_low_lane], '--k', 'LineWidth', 1);
    hold off;
end
function plotVehicles()
    global Xv1;
    global Xv2;
    global l;
    global b;     
    figure(2);               
    hold on;
    box on;
    rectangle('Position', [Xv1(end,1)-l/2 Xv1(end,2)-b/2 l b], 'FaceColor', 'r');
    rectangle('Position', [Xv2(end,1)-l/2 Xv2(end,2)-b/2 l b], 'FaceColor', 'b');
    hold off
end
