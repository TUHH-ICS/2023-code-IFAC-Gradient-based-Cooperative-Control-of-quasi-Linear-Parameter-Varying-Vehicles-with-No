%---------------------------------------------------------------------------------------------------
% For Paper
% "Gradient-based Cooperative Control ofquasi-Linear Parameter Varying Vehicleswith Noisy Gradients"
% by Adwait Datar, Antonio Mendez Gonzalez and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Antonio Mendez Gonzalez
%---------------------------------------------------------------------------------------------------
% This function generates Figure 1 of the paper, where
% L and delta are varired
% Convergence rates for different f in S(1,L) are guaranteed by:
% 1st order ZF multipliers and Kd = 20

%% Setup
constants = loadConstants();        %Load general constants

ZFOrder  = 5;                       %Order for the ZF Multiplier
m_sml    = 1;                       %m of f in S(m,L)
d        = 1;                       %number of spatial dimensions
alphaMax = 0.05;                    %upper bound for alpha in the bisection algorithm

constants.nu    = ZFOrder;
constants.m_sml = m_sml;

barH = waitbar(0,'Progress: 0%');

%The LPV Controller and Closed Loop System are loaded
%G_veh is the Closed Loop System, i.e. the feedback interconnection between 
%the LPV Controller and Plant; it is used in the analysis algorithm.
%The LPV Controller is used in the simulation.
%Other relevant variables are also loaded
load G_NLF.mat

%% Gpq
% The Gpq system is defined

Kp = 1;         %Kp gain of the Gpq system
Kd = 20;        %Kd gain of the Gpq system

%System matrices of the Gpq system
Apq = [ zeros(d)  eye(d)    ;
        zeros(d) -Kd*eye(d) ];

Bpq = [  zeros(d)  ;
        -Kp*eye(d) ];

Cpq = [ eye(d) zeros(d) ];

Dpq = zeros(d);

Gpq = ss(Apq, Bpq, Cpq, Dpq);

%% Construction of G, G1 and G2 

G = Gveh*Gpq;

%G1 and G2 are defined by the extreme points of the LPV parameters
G1 = usubs(G, 'f', fminmax(1));
G2 = usubs(G, 'f', fminmax(2));

%% Sweep values
delta = [0 0.01 0.05 0.1:0.2:0.9 0.95];
L = [1:0.5:17];