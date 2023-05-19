function LC = TTC_TIV_1(Dist, Vdiff, v1, v2, lane1, lane2)
% is lane changing feasible
% LC: a logical output
% dist = ego - vehicle
% vdiff = v_ego -v_vehicle
% v1: v_vehicle
% v2: v_ego
% lane1: vehicle lane
% lane2: ego lane

TTCth = 4; % TTC threhold
TIVth = 1; % TIV threhold

% preprocess Dist and Vdiff 
Dist = Dist*( abs(Dist) > 4.7);
Vdiff = Vdiff*(abs(Vdiff) > 0.5);

if lane2  == 3
    TTCth = 6;
    TIVth = 1.5;
elseif lane2  == 1
    TIVth = 1.5;
end

if Dist > 0 % ego in fornt of vehicle
   % TTC
    if Vdiff >= 0
        TTC = Inf;
    else
        TTC = abs(Dist/Vdiff);
    end
   % TIV 
    TIV = abs(Dist)/v1;
elseif Dist ==0 % ego and vehicle parallel
        TTC = 0;
        TIV = 0;
else            % ego in behind of vehicle
    if Vdiff <= 0
        TTC = Inf;
    else
        TTC = abs(Dist/Vdiff);
    end
    TIV = abs(Dist)/v2;
end

LC = true;
if TTC <= TTCth
    LC = false;
else
    if TIV <= TIVth
        LC = false;
    else
        LC = true;
    end
end


end