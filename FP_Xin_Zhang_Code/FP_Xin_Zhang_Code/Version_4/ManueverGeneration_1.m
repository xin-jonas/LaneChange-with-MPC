function tempx = ManueverGeneration_1(x,kT,Xv2,Vv2)
% maneuvering planning and determine the reference for cost calculation
w_lane = 5.25; % lane breite
b = 1.83; % vehicle breite
%%  vehicles manuever generation
%    left keep right
% de  1    1     1
% cs  1    1     1
% ac  1    1     1
maneuver = ones(3,3);

%% kT -> x ; Xv2 -> before kT
% x4_constraint = 3*w_lane/2;
% x4_weight = 10;
% x1_constraint = 30;
% x1_weight = 100;

%% ego information
ego = cell(1,3);
ego{1,1} = laneDetermine(x(4));
ego{1,2} = x(1);
%% vehicles
V = {};
vehicles = cell(1,4);
% 1: lane number
% 2: velocity in x 
% 3: predictive distance with Ego in kT
% 4: velocity difference
vehicles{1,1} = laneDetermine(Xv2(end,2));
vehicles{1,2} = Vv2(end,1);
vehicles{1,3} = x(3)-(Xv2(end,1)+Vv2(end,1)*kT); 
vehicles{1,4} = x(1)-Vv2(end,1);
V{1,1} = vehicles;

%% car in absolute safe distance area
car = check_area(V);
ego{1,3} = car;

%% manuever choose
if isempty(car)
    %disp('situation1_empty');
    % no car in consider area
    % car keep lane with recommended velocity
    if ego{1,1} == 3 % if car in lane 3, go to lane 2 
        x1_ref = 30;
        x4_ref = 3*w_lane/2;
    elseif ego{1,1} == 2 % if car in lane 2, keep in lane 2
        x1_ref = 30;
        x4_ref = 3*w_lane/2;
    else                % if car in lane 1, keep in lane 1
        x1_ref = 30;
        x4_ref = w_lane/2;
    end
    x1_weight = 10;
    x4_weight = 100;
    tempx = {x1_weight x4_weight x1_ref x4_ref};
else
    %disp('situation2_not_empty');
    % target vehicle in consider area
    % manuever selction
    for i = 1:size(car)
        v = car{1,i};
        % TL: trun left, LK: lane keep, TR: turn right
        [TL, LK, TR] = position_car(ego, v); % possible Manuever generation
        %disp('TL')
        %disp(TL);
        if LK == 1
            tempx = laneKeep(ego{1,1},vehicles{1,3},vehicles{1,2}); % car keep lane with recommended velocity
        elseif TR == 1  % only if ego in lane 3 and vehilce in lane 1
            x1_ref = 30;
            x4_ref = 3*w_lane/2;
            x1_weight = 10;
            x4_weight = 100;
            tempx = {x1_weight x4_weight x1_ref x4_ref};
            % disp('TR');
        else % LK or LC
            % determine one manuever based TTC and TIV
            LC = TTC_TIV_1(v{1,3}, v{1,4}, v{1,2}, x(1),v{1,1},ego{1,1});
            %disp('LC');
            %disp(LC);
            if LC
                if TL > 0
                   % TL = 1;
                    maneuver(:,2) = 0;
                    maneuver(:,3) = 0;
                elseif TR > 0
                    %TR = 1;
                    maneuver(:,1) = 0;
                    maneuver(:,2) = 0;
                end
            else
                %LK = 1;
                maneuver(:,1) = 0;
                maneuver(:,3) = 0;
            end   
            [x1_weight, x1_constraint] = VelocitySet(LC, v{1,3}, v{1,2}, v{1,1},ego{1,1});
            % choose one manuever
            c=chooseOneMan(maneuver);
            % set weihgt and reference
            [x4_weight, x4_constraint] = setParameter(c,ego{1,1},x(1),Vv2(end,1));
            tempx = {x1_weight x4_weight x1_constraint x4_constraint};

        end
    end
end
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% lane determine
    function laneNr = laneDetermine(x)
    if abs(x-w_lane/2) < (w_lane/2 )
        laneNr = 1;
    elseif abs(x-3*w_lane/2) < (w_lane/2 ) 
        laneNr = 2;
    elseif abs(x-5*w_lane/2) < (w_lane/2)
        laneNr = 3;
    end
 end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% detect car in abs safe area
    function Car_detected = check_area(V)
        Car_detected = {};
        a = 1;
        % experience value: safe Distance = Tacho_Velocity/2
        AbSafeDist = 36*3.6/2; % consider max velocity
        for i = 1: size(V,2)
            if abs(V{1,i}{1,3}) < AbSafeDist
                Car_detected{1,a} = V{1,i};
                a = a + 1;
            end
        end
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% car relative position to determine the prioity of the LC or LK
% SV: subject vehicle
% Vd: determinate Vehicle
    function [LF, LK, LR] = position_car(SV, Vd) 
        % laneNr = Vd{1,1}
        if SV{1,1} == 1 % ego in lane 1
            switch Vd{1,1} % determinate Vehicle lane Nr.
                case 3
                    LF = 0;
                    LR = 0;
                    LK = 1;
                otherwise
                    if Vd{1,3} < 0 % determinate Vehicle is in fornt of ego
                        LF = 0.5;
                        LR = 0;
                        LK = 0.5;
                    else
                        LF = 0;
                        LR = 0;
                        LK = 1;
                    end
            end
        elseif SV{1,1} == 2
            switch Vd{1,1}
                 case 2
                    if Vd{1,3} < 0
                        LF = 0.5;
                        LR = 0;
                        LK = 0.5;
                    else
                        LF = 0;
                        LR = 0;
                        LK = 1;
                    end
                otherwise
                    LF = 0;
                    LR = 0;
                    LK = 1;
            end
        else
            switch Vd{1,1}
                case 2
                    LF = 0;
                    LR = 0.5;
                    LK = 0.5;
                otherwise
                    LF = 0;
                    LR = 1;
                    LK = 0;
            end
                    
        end
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% lane keep if LK == 1
    function tempx = laneKeep(lane_ego,Dist, V1)
        if ego{1,1} == 3
            if Dist > 0
                x1_ref = max([V1,36]);
            else
                x1_ref = min([36,V1*0.95]);
            end
                x4_ref = 5*w_lane/2;
        elseif ego{1,1} == 2
            if Dist > 0
                x1_ref = max([V1,30]);
            else
                x1_ref = min([30,V1*0.9]);
            end
            x4_ref = 3*w_lane/2;
        else
            if Dist > 0
                x1_ref = max([V1,30]);
            else
                x1_ref = min([30,V1*0.95]);
            end
            x4_ref = w_lane/2;
        end
        x1_weight = 10;
        x4_weight = 100;
        tempx = {x1_weight x4_weight x1_ref x4_ref};
    end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function y=chooseOneMan(maneuver)
       [s,t] = find(maneuver==1); % speed de cs ac, turn: left keep right
            % lane change or lane keep
            if ismember(1,t)
                y=1;
            elseif ismember(2,t)
                y=2;
            else
                y=3;
            end 
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function [x4_weight, x4_constraint] = setParameter(y,lane1,v_ego,v2)
        % x: velocity change
        % y: lane change or lane keep
        % lane1: ego vehicle lane 
        % v_ego: velocity of ego
        % v2: velocity of target vehicle
        if y==1  % trun left
            switch lane1
                case 1
                  x4_constraint = 3*w_lane/2; % go to 2. lane 
                  x4_weight = 100;
                case 2
                   x4_constraint = 5*w_lane/2; 
                   x4_weight = 100; 
            end
        elseif y==2 % lane keep
            switch lane1
                case 1
                   x4_constraint = w_lane/2; 
                   x4_weight = 100;
                case 2
                   x4_constraint = 3*w_lane/2; % go to 1. lane 
                   x4_weight = 100;
                case 3
                   x4_constraint = 5*w_lane/2; % 
                   x4_weight = 100;
            end   
        else    % turn right
            switch lane1
                case 1
                   x4_constraint = w_lane/2; % 
                   x4_weight = 100;
                case 2
                   x4_constraint = w_lane/2; % 
                   x4_weight = 100;
                case 3
                   x4_constraint = 3*w_lane/2; % 
                   x4_weight = 100;
            end   
        end % y end  
        
    end % function end
end


