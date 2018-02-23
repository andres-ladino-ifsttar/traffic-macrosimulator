%% Single example on how to obtain the control action
%
% This file solves the optimization problem proposed in 
% <part_3_control_centralized.html Control Centralized Algorithm>. The 
% solution is found via <http://yalmip.github.io/tutorial/basics/ YALMIP>
% 
%% Preparing everything beforehand
%  
% Before starting be sure to have the correct setup. For that, 
% 
% * Follow instructions in <part_0_setup.html Setup>.
%
% Add the folder of functions first 
pathEnabler('activate')
%% Parameter setup
%
% Setting up the parameters. The example is developed over the *Manhattan*
% case. Please adapt the parameters to the corresponding network you are analizing. 
nRows                   = 4; 
nCols                   = 4;
[incMat, adjMat]        = buildManhattanMultipleInOut(nRows,nCols);
nRoads                  = size(incMat,1);
Ts                      = 15;               % sampling time
Tc                      = 90;               % cycle time of traffic lights 
T                       = Tc/Ts;            % cycle time/ sampling time
Li                      = 500;              % cell length [m]
maxSpeed                = 50/3.6;           % free flow speed [m/s]
maxDensity              = 0.125;            % maximum density [veh/m]
maxFlow                 = 0.55;             % maximum flow [veh/s]
rhoC                    = maxFlow/maxSpeed; % critical density
congSpeed               = abs(maxFlow/(rhoC - maxDensity));   % w
type                    = 'RoadSignFifoCTM';% model type
netControlSCTM          = Net(Ts, T, (1:nRoads)');
netControlSCTM.iM       = incMat;
netControlSCTM.turnings = adjMat;
%%
% Set parameters to the network 
netControlSCTM.initialize(Li, maxSpeed, congSpeed, maxDensity, ... 
                        maxFlow, rhoC, type)
%%
% Setup the flag so the network is run under regulated control control.
% Lights are going to be setup since it is just inside the |Net| model that
% the automatic control signal is selected.
netControlSCTM.isControlled= 1;
setTrafficLightsManhattanMultipleIO(netControlSCTM,nCols); 
%
% _Note: This can be verified in the simul method in the_ |Net| _class_
%%
% Setup for simulation parameters: 
nCycles                 = 50;
time                    = [1 T*nCycles]; % number of iterations
Sout                    = maxFlow*ones(nRoads,time(end));
%
burstValue              = 0.4; 
t0                      = time(end)*0.1; 
tf                      = time(end)*0.8;
Din                     = inputDemandManhattan(netControlSCTM, burstValue,...
                                               t0, tf, time(end));      
%% Capture a set of measurements 
%
% This input should come from the simulator, in this case we emulate a
% measurement. 
y_density   = rand(nRoads,1)*netControlSCTM.roads(1).maxDensity; % To recover from  the real system/simulator
%% Compute the control
% 
% Compute the control action through the function _CentralizedMPC_
% 
% Let suppose at time instant $k$ (beguining of the cycle) 
k           = 25*netControlSCTM.period
%%
% Retrieve the external demand/supply for during the period. 
DinCycle    = Din(:,k:k+netControlSCTM.period-1);
SoutCycle   = Sout(:,k:k+netControlSCTM.period-1);
%% 
[LightsValues, xPred, fOut, fIn, TTD_val, duty, objective] = ...
         CentralizedMPC(netControlSCTM, DinCycle, SoutCycle,y_density)
%% Examining the results  
% 
% In this case the variable |LightsValues| contains the value to be applied 
% during the next cycle. 
LightsValues % Value to apply to each light on the system
%%
% The corresponding duty cycle is stored in |duty|
duty % Solution of the optimal control problem
%%
% Exam _help(CentralizedMPC)_ in order to obtain more informations about
% other indicators


     