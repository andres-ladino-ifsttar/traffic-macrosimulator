%% ITS Session 2: Control of large scale Traffic Networks 
%
% The course is oriented to introduce the models presented in the first
% session. 
%% Previous information 
% 
% This session is based on contents on _Control of large scale urban 
% traffic networks_. Check slides at: 
%%
% *  <http://bit.ly/ITS-Control-Session-1 Slides Session 1>
%% Session content 
% 
% The session is divided in multiple areas. Click on each link to get
% deeper information on each one of the subjects:
%%
% 1. <part_0_setup.html Setup>: Initial steps before starting.
%%
% 2. <part_1_modeling_sctm.html Modeling S-CTM>: Signalized cell
% transmission model (with manual set lights) 
%%
% 3. <part_2_modeling_actm.html Modeling Avg-CTM>: Averaged cell
% transmission model (with manual set lights) 
%%
% 3. <part_3_control_centralized.html Control Centralized Algorithm>:
% Quadratic programming one step ahead controller
%% 
%% About this tool 
% 
% _By P. Grandinetti_
% 
% This is a MatLab package that can be used to create and simulate road 
% networks objects described by a macroscopic dynamics. The code comes 
% entirely from the legacy code developed during the Ph.D. of the author; 
% all technical details can be checked in the Ph.D.
% <bit.ly/TrafficControl-PhD Thesis available>.
%%
% <https://en.wikipedia.org/wiki/Reproducibility#Reproducible_research 
% Reproducibility> is an important feature, sadly often neglected in 
% published works. When using this repository, other than checking the 
% above mentioned thesis for a deeper insight, we provide two fully 
% reproducibile examples (with images generation), to ease the 
% understanding of the code. Thus, in the folder `reproducible example` 
% you will find
%
% * A runnable script to simulate one road network and instructions 
% about how to set up your experiments,
% * A runnable script to simulate two different networks, one with 
% optimized traffic lights and one without; the script produces several 
% figures in order to compare the systems' performance.
%% 
% The above mentioned Ph.D. work was about _Control of large scale traffic
% networks_. Differently said, our objective was not just to simulate 
% dynamic networks, but also to design algorithms to improve the behavior, 
% for instance by reducing the congestions; the developed controllers are 
% stored in the subfolder |controllers|.
%% Requirements
% 
% * Any recent version of MatLab should work
% * <https://yalmip.github.io/ YALMIP>
% & Some good numerical solvers: most of the algorithms are based on convex 
% optimization, for which you can use built-in solvers, or external solvers 
% like cvx, mosek, etc. Other controllers are instead based on integer 
% optimization, therefore an appropriate solver is needed (here again, 
% mosek will work).
%% Examples
%
% When running the |networkComparisons| script, you will see the following 
% two figures (among others)
%%
% 
% <<controlled.png>>
% 
%%
% 
% <<uncontrolled.png>>
% 
%%
% The figures show how the density of vehicles in every road 
% (with roads indexes from 1 to 40 in the y-axis) evolves along the entire 
% simulation time (x-axis). By comparing the two, it is possible to see the 
% the controller spreads out the severe congestions (that appear in the 
% uncontrolled case) by creating a more homogeneous distribution of vehicles 
% all over the network.

%% Contributors
%
% The code has been entirely written by Pietro Grandinetti. Adaptation of
% this tool for the ITS Session has been done by Andres Ladino.

%%
pathEnabler('activate')
part_1_modeling_sctm
part_2_modeling_actm
part_3_control_centralized

