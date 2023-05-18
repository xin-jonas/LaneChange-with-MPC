function [x1_weight, x1_constraint] = VelocitySet(LC, Dist, V1, lane1, lane2)
% set goal velocity based on goal lane and velocity of target vehilce
% LC: the output of function TTC_TIV_1
% Dist: x_ego - x_v
% V1: velocity of target vehicle
% lane1: lane of target vehicle
% lane2: lane of ego vehicle

x1_weight = 10;
if lane2 == 1
    if lane1 == 1
    %% Dist > 0, ego in behind, lane1 == lane2
        if LC
            if abs((V1-30)/30) < 0.05
                x1_constraint = max([30*1.2,V1*1.15]);
            else
                x1_constraint = max([30,V1*1.1]);
            end
        else
            x1_constraint = min([30,V1*0.95]);
        end
    else        % lane1 == 2
        if LC
            if abs((V1-30)/30) < 0.05
                x1_constraint = max([30*1.2,V1*1.15]);
            else
                x1_constraint = min([30,V1]);
            end
        else
            if abs((V1-30)/30) < 0.15
                x1_constraint = min([30,V1*0.95]);
            else
                x1_constraint = 30;
            end
                
        end
    end
% target vehicle and ego vehicle both are in 2 second lane
elseif lane2 == 2 
        if LC
            x1_constraint = max([30*1.2,V1*1.2]);
        else
            x1_constraint = min([30,V1*0.95]);   
        end
else
% ego vehicle in 3. lane
    if lane1 == 3 % target vehicle in 3. lane
        if LC
            if Dist > 0
                x1_constraint = max([30,V1*0.95]);
            else
                x1_constraint = min([30,V1*0.85]);
            end
        else
            if Dist > 0
                x1_constraint = max([36,V1]);
            else
                x1_constraint = min([36,V1]);
            end
        end
    else % target vehicle in 2. lane
        if LC
            if Dist > 0
                x1_constraint = 30;
            else
                x1_constraint = min([30, V1]);
            end
        else
           x1_constraint = 36;
        end
    end
end
end