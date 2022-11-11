%---------------------------------------------------------------------------------------------------
% For Paper
% "Gradient-based Cooperative Control ofquasi-Linear Parameter Varying Vehicleswith Noisy Gradients"
% by Adwait Datar, Antonio Mendez Gonzalez and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Antonio Mendez Gonzalez
%---------------------------------------------------------------------------------------------------
% This function generates Figure 5 of the paper, where
% L and delta are varired
% Convergence rates for different f in S(1,L) are guaranteed by:
% 5th order ZF multipliers and Kd = 20

%% Setup
constants = loadConstants();        %Load general constants

ZFOrder     = 5;                    %Order for the ZF Multiplier
m_sml       = 1;                    %m of f in S(m,L)
alphaMax    = 0.05;                 %upper bound for alpha in the bisection algorithm (lower bound is 0 by default)

gridL = [1:30:100];                %grid for L
gridDelta = [0 0.3 0.5 .9];         %grid for delta

%add values to constants
constants.nu        = ZFOrder;
constants.m_sml     = m_sml;
constants.alphaMax  = alphaMax;

%Progress bar
barH = waitbar(0,'Progress: 0%');
progress = 0;
progressEND = length(gridL)*length(gridDelta);

%The LPV Controller and Closed Loop System are loaded
%G_veh is the Closed Loop System, i.e. the feedback interconnection between 
%the LPV Controller and Plant; it is used in the analysis algorithm.
%The LPV Controller is used in the simulation.
%Other relevant variables are also loaded
load G_NLF.mat

%% Gpq
% The Gpq system is defined
d  = 1;         %number of spatial dimensions for each agent
Kp = 1;         %Kp gain of the Gpq system
Kd = 20;        %Kd gain of the Gpq system

Gpq = defineGpq(d,Kp,Kd);

%% Construction of G, G1 and G2 

G = Gveh*Gpq;

%G1 and G2 are defined by the extreme points of the LPV parameters
G1 = usubs(G, 'f', fminmax(1));
G2 = usubs(G, 'f', fminmax(2));


%% Define the system

system.Kp   = Kp;           %Kp from Gpq
system.Kd   = Kd;           %Kd from Gpq
system.G    = G;            %G = Gveh*Gpq;
system.G1   = G1;           %G1 evaluated at min(LPVparameter)
system.G2   = G2;           %G2 evaluated at max(LPVparameter)
system.d    = d;            %number of dimensions
%notice that further modifications are needed for more LPV parameters

%% Grid
Data = {};
for delta_i = gridDelta
    for L_i = gridL
        progress = progress + 1;                %For progress bar

        constants.L_sml = L_i;                  %Grid point of L
        constants.delta = delta_i;              %Grid point of delta

        [success, results] = findBestAlpha(system, constants);         %perform bisection for this grid point

        Test.success = success;
        Test.results = results;
        Test.constants = constants;
        Test.delta = delta_i;

        Data{end+1} = Test;                     %Store variables

        barProgress = progress/progressEND;     %Update progress bar
        barMsg = ['Almost there... ' num2str(barProgress*100) '%'  '  [d L] = ' num2str([delta_i L_i])];
        waitbar(barProgress, barH, barMsg);
    end
end

%% Plot
plotFigure5
