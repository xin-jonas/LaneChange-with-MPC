%% Cost follow Generation
function cost = runningcosts1v21(t,x,u,kT,tempx)
 global Xv1;
 global Vv1;
 global Xv2;
 global Vv2;
 global x_safe;
 global x1_w;
 global x4_w;
 global x1_c;
 global x4_c;
%maneuver update at each iteration
% tempx: generate from the ManueverGeneration_1.m, which is in nmpc.m called 
[x1_w, x4_w, x1_c, x4_c] = tempx{:};
cost = 1*u(1)^2 + 0.1*u(2)^2 + x1_w*(x(1) - x1_c)^2 + x4_w*(x(4)-x4_c)^2;

end
