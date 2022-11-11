%---------------------------------------------------------------------------------------------------
% For Paper
% "Gradient-based Cooperative Control ofquasi-Linear Parameter Varying Vehicleswith Noisy Gradients"
% by Adwait Datar, Antonio Mendez Gonzalez and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Antonio Mendez Gonzalez
%---------------------------------------------------------------------------------------------------
% This function generates Figure 7 of the paper, where
% A simulation of a single agent in two scenarios:
% Stable and unstable

%% Setup

N = 1;              %1 Agent

fRef = [1:N]*0;     %Formation reference is irrelevant for 1 Agent
Lap  = 0;           %Laplacain is irrelevant for 1 Agent

IC = [250.001 0];   %Nearby source (Initial conditions 0 speed)

grad_sel = 1;       %the agent receives the gradient

Kp = 1;             %Kp and Kd for Gpq
Kd = 18.5;

b = 0.01;           %b and m for the non linear system
m = 1;

deltaNoise = 0.5;   %Level delta for noise

load G_NLF.mat      %load the controller

xM = 250;           %Source of the field
L_psi = 70;         %L_psi of the field

%the field is quadratic and has the form 0.5*x^2*L_psi,
%i.e. the gradient is L_psi*x (see corresponding simulink block)

fieldParams = [];
fieldParams.xM      = xM;
fieldParams.L_psi   = L_psi;

load BusSim.mat     %load a bus for simulink

%% Simulation
simulationFile = 'MAS_All.slx';

%1st simulation results
sim1_unstable = sim(simulationFile); 

%2nd simulation results (Kd optimized for delta = 0.5)
Kd = 29;
IC = [251.5 0];   %Nearby source

sim2_stable = sim(simulationFile); 
%% Plot data
plotFigure7