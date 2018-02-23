%% ITSII - Externalized controller 
% 
% Call this function in this way:
% 
% [LightsValues, xPred, fOut, fIn, flows, duty, objective] = OneStepAheadCentralizedQP(net, Din, Sout)
%
% This file has been implemented for control implementation outside of
% Traffic MacroSimulator. 
% 
% This function implements a discrete time controller for traffic lights
% See "Control of large scale traffic networks" by the P. Grandinetti,
% Chapter 5
% 
% See references also in:
% 
% Inputs: net : Network object. Consider first the inizialization of the
%               network. Run before 
%               Assign the incidence matrix : net.iM
%               Assign the adjacency matrix: net.turnings
%               Initialize network parameters: net.initialize
%         Din : External demand matrix 
%               Matrix [n x m] 
%                   n: number of roads  
%                   m: time 
%               Din should be 0 forall m in internal/exiting roads, and any value 
%               for all time entering roads.
%               time - corresponds to the Demands from k to k+Tc where Tc
%               is the cycle time
%         Sout: External supply matrix
%               Matrix [n x m] 
%                   n: number of roads  
%                   m: time 
%               Sout should be 0 forall m in internal/entering roads, and any value 
%               for all m in exiting roads. 
%               time - corresponds to the Supplies from k to k+Tc where Tc
%               is the cycle time
%         y_density: Density measurements vector:
%               Vector [n x 1] 
%         
%
% Outputs: 
%         LightsValues: Solution of the optimization problem
%
%         Xpred: Predicted state at step k+1 (Densities)
% 
%         fOut: Outflow computation. See 
%           
%         fIn: Inflow computation. See 
% 
%         TTD_val: Value of TTD after the optimization
%  
%         Objective: Value of Balancing + TTD after optimization

function [LightsValues, xPred, fOut, fIn, TTD_val, duty, objective] = ...
         CentralizedMPC(net, Din, Sout,y_density)
    yalmip('clear')

    duty        = sdpvar(length(net.lM),1);
    duty0       = net.getDutyCycles();
    F           = [duty(net.exitingRoads)==1];
    F           = F + [duty>=0.1, duty <=1];
    
    DinVar      = mean(Din,2);
    SoutVar     = mean(Sout,2);

    fOut        = sdpvar(length(net.roads),1);
    fIn         = sdpvar(length(net.roads),1);

    demands     = zeros(length(net.roads),1);
    supplies    = zeros(length(net.roads),1);

    % xCurr       = [net.roads(:).currentDensity];
    xCurr       = y_density';

    for i = 1 : length(net.roads)
        demands(i)  = min(net.roads(i).maxSpeed * xCurr(i),...
                          net.roads(i).maxFlow);
        supplies(i) = min (net.roads(i).maxFlow, ...
                      net.roads(i).congSpeed * (net.roads(i).maxDensity - xCurr(i)));
    end

    for i = 1 : length(net.roads)
        outRoads = net.neighborsOut(i);
        if ~isempty(outRoads)
            fOut(i) = duty(net.lM==i) * min([demands(i); (supplies(outRoads)./net.turnings(i,outRoads)')]);
        else
            fOut(i) = duty(net.lM==i) * min(demands(i), SoutVar(i));
        end
    end

    for i = 1 : length(net.roads)
        inRoads = net.neighborsIn(i);
        if ~isempty(inRoads)
            fIn(i) = dot( net.turnings(inRoads,i), fOut(inRoads));
        else
            fIn(i) = min(DinVar(i), supplies(i));
        end
    end

    for i = 1 : size(net.iM , 2)
        kin = net.kinIntersectionDown(i);
        if ~isempty(kin)
            F = F + [sum(duty(kin))==1];
        end
    end

    xPred = xCurr' + net.sampleTime./[net.roads(:).L]' .* (fIn - fOut);
    bal = 0;
    for i = 1 : length(net.roads)
        iNeighDown = net.neighborsOut(i);
        for j = 1 : length(iNeighDown)
            tmp = sdpvar(1);
            F = F + [tmp == xPred(i) - xPred(iNeighDown(j))];
            bal = bal + tmp^2;
            %bal = bal + tmp^2/(net.roads(i).maxDensity*0.5);
            %bal = bal + (xPred(i) - xPred(iNeighDown(j)))^2;
        end
    end

    %innerR = net.innerRoads();
    %factor = sum([net.roads(:).maxFlow]);
    TTD = min ( net.roads(1).maxSpeed * xPred, ...
        net.roads(1).congSpeed*(net.roads(1).maxDensity - xPred) ); % TTD
    %obj = bal- sum(TTD./([net.roads(:).maxFlow]')) + (duty-duty0)'*(duty-duty0);
    %obj = bal- sum(TTD./[net.roads(:).maxFlow]');
    %obj = - sum(TTD) + (duty-duty0)'*(duty-duty0);
    
    obj = bal/net.roads(1).maxDensity- sum(TTD)/net.roads(1).maxFlow;% + (duty-duty0)'*(duty-duty0);
    sol = optimize(F, obj, sdpsettings('verbose', 0)); % 'solver', 'mosek', 
    if sol.problem ~= 0
        disp(sol)
        error('Numerical error while optimizing')
    end
    duty = value(duty);
    LightsValues = convertDutyCycles(net, duty);
    xPred = value(xPred);
    fIn = value(fIn);
    fOut = value(fOut);
    TTD_val = value(TTD);
    objective = [value(sum(sum(bal))) value(sum(sum(TTD)))];

end