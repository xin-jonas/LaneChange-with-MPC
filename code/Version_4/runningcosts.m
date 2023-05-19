%% Cost follow Generation
function cost = runningcosts(t,x,u,kT)
 global Xv1;
 global Vv1;
 global Xv2;
 global Vv2;
 global x1_w;
 global x4_w;
 global x1_c;
 global x4_c;
% first mod(a,b): a: iteration steps, b: parameter which can be changed to set
% update frequence
% second mod(a,b): a: prediction time, b:great than 25*0.4, so the result
% equals zero only when prediction time is zero
if mod(int32(t/0.2),2) == 0  && mod(kT,5) <= 1e-5 
    tempx = ManueverGeneration_1(x,kT,Xv2,Vv2);
    [x1_w, x4_w, x1_c, x4_c] = tempx{:};
end
cost = 1*u(1)^2 + 0.1*u(2)^2 + x1_w*(x(1) - x1_c)^2 + x4_w*(x(4)-x4_c)^2;
end
