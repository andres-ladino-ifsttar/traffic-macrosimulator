%% ITS Session 2: Control of large scale Traffic Networks 
% 
%% 1. Preparation & Setup
% 
% This module is a basic guide for YALMIP installation. YALMIP is a
% software to parse optimization problems. More information here (
% <http://yalmip.github.io/tutorial/basics/ YALMIP> ). This sofware is
% used as part of the controller design problem.
%
%% 1.1 Installing Yalmip
%
% First before anything be sure to install YALMIP and verify the correct
% installation by runing:
%
% 1. Inside the matlab folder _MATLAB_ create a folder named _YalmipToolbox_
% 2. Execute the installation by 
%
cd YalmipToolbox
urlwrite('https://github.com/yalmip/yalmip/archive/master.zip','yalmip.zip');
unzip('yalmip.zip','yalmip')
addpath(genpath([pwd filesep 'yalmip']));
savepath
%%
% 3. Check installation procedure by executing the following line and
% verify the answer message provided by YALMIP:
%
yalmiptest
%
% Yalmip output: 
%
% |++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% |                   Test|   Solution|                          Solver message|
% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% |   Core functionalities|        N/A|            Successfully solved (YALMIP)|
% |                     LP|    Correct|   Successfully solved (GLPK-GLPKMEX-CC)|
% |                     LP|    Correct|   Successfully solved (GLPK-GLPKMEX-CC)|
% |                     QP|    Correct|          Successfully solved (QUADPROG)|
% |                     QP|    Correct|          Successfully solved (QUADPROG)|
% |                   SOCP|    Correct|        Successfully solved (SeDuMi-1.3)|
% |                   SOCP|    Correct|        Successfully solved (SeDuMi-1.3)|
% |                   SOCP|    Correct|        Successfully solved (SeDuMi-1.3)|
% |                    SDP|    Correct|        Successfully solved (SeDuMi-1.3)|
% |                    SDP|    Correct|        Successfully solved (SeDuMi-1.3)|
% |                    SDP|    Correct|        Successfully solved (SeDuMi-1.3)|
% |                    SDP|    Correct|        Successfully solved (SeDuMi-1.3)|
% |                 MAXDET|    Correct|        Successfully solved (SeDuMi-1.3)|
% |                 MAXDET|    Correct|        Successfully solved (SeDuMi-1.3)|
% |          Infeasible LP|        N/A|    Infeasible problem (GLPK-GLPKMEX-CC)|
% |          Infeasible QP|        N/A|           Infeasible problem (QUADPROG)|
% |         Infeasible SDP|        N/A|         Infeasible problem (SeDuMi-1.3)|
% |      Moment relaxation|    Correct|        Successfully solved (SeDuMi-1.3)|
% |         Sum-of-squares|    Correct|        Successfully solved (SeDuMi-1.3)|
% |           Bilinear SDP|        N/A|                      No suitable solver|
% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++|

%% 1.2 Structure of simulation files
%
% Code for this demo can be obtained at:
%
%%
% * <http://www.github.com/andres-ladino-ifsttar/traffic-macrosimulator GitHub Traffic Macrosimulator>
%
%%
% The compressed file is composed by 4 main folders:
% 
% * _reproducibleExample_ : Reproduces exact simulations explained in
% |README.md|. 
% * _networkShapes_ : Contains functions for traffic network creation and
% visualization tools.
% * _controllers_ : Contains a variety of controllers for the traffic
% network. 
% * _sessionLab_ : Files for step by step demonstration purposes.  
%
% Decompress the .zip file or in a folder and go to the MATLAB directory
% and execute 
%
pathEnabler('activate')
%%
% This function will activate all required paths required for the work
% the session. 
