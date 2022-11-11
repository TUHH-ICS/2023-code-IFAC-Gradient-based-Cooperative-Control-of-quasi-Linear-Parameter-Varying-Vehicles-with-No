%---------------------------------------------------------------------------------------------------
% For Paper
% "Gradient-based Cooperative Control ofquasi-Linear Parameter Varying Vehicleswith Noisy Gradients"
% by Adwait Datar, Antonio Mendez Gonzalez and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Antonio Mendez Gonzalez
%---------------------------------------------------------------------------------------------------
% This function generates Figure 8 of the paper, where
% A simulation of 10 agents reaching the source

%% Setup

N = 10;              %1 Agent

fRef = [1:N]*0;     %Formation reference = 0, i.e. consensus (to avoid conflict with gradient)

%Cycle for the laplacian in 10 agents
Lap = [ 2 -1  0  0  0  0  0  0  0 -1 ;
       -1  2 -1  0  0  0  0  0  0  0 ;
        0 -1  2 -1  0  0  0  0  0  0 ;
        0  0 -1  2 -1  0  0  0  0  0 ;
        0  0  0 -1  2 -1  0  0  0  0 ;
        0  0  0  0 -1  2 -1  0  0  0 ;
        0  0  0  0  0 -1  2 -1  0  0 ;
        0  0  0  0  0  0 -1  2 -1  0 ;
        0  0  0  0  0  0  0 -1  2 -1 ;
       -1  0  0  0  0  0  0  0 -1  2 ];


IC = [[245:249 251:255]' 0*[1:N]'];   %Initial conditions Nearby source (0 speed)

grad_sel(1:10) = 1;       %all agents receive the gradient

Kp = 1;             %Kp and Kd for Gpq
Kd = 100;

b = 0.01;           %b and m for the non linear system
m = 1;

deltaNoise = 0.5;   %Level delta for noise

load G_NLF.mat      %load the controller

xM = 250;           %Source of the field
L_psi = 66;         %L_psi of the field

%the field is quadratic and has the form 0.5*x^2*L_psi,
%i.e. the gradient is L_psi*x (see corresponding simulink block)

fieldParams = [];
fieldParams.xM      = xM;
fieldParams.L_psi   = L_psi;

load BusSim.mat     %load a bus for simulink

GLbig = Lap + diag(grad_sel)*L_psi;

GLsmall = Lap + diag(grad_sel)*1;
disp('S(m,L), based on ground Laplacians')
m = min(eig(GLsmall))
L = max(eig(GLbig))



%% Simulation
simulationFile = 'MAS_All.slx';

%1st simulation results
sim1_MAS = sim(simulationFile); 
%% Plot data
plotFigure8
