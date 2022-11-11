%---------------------------------------------------------------------------------------------------
% For Paper
% "Gradient-based Cooperative Control ofquasi-Linear Parameter Varying Vehicleswith Noisy Gradients"
% by Adwait Datar, Antonio Mendez Gonzalez and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Antonio Mendez Gonzalez
%---------------------------------------------------------------------------------------------------
% This function loads some constants that are used by the solver and the analysis algorithm.

function constants = loadConstants()

%% LMI Solver constants
LMIsolver = 'sdpt3'; %Solver used for the optimization

%Numerical setting for SDPT3
SDPoptions.sdpt3.rmdepconstr    = 1;
SDPoptions.sdpt3.allownonconvex = 0;
SDPoptions.sdpt3.maxit          = 800;
SDPoptions.sdpt3.gaptol         = 1e-5;
SDPoptions.sdpt3.inftol         = 1e-5;
SDPoptions.sdpt3.steptol        = 1e-5;
SDPoptions.sdpt3.scale_data     = 1;

% Yalmip: Radius constraint on all pimal variables ||x||<radius
radius = 1e9;

dB = 1e-6;          %Boundary for positive/negative definitness

%% Feasibility algorithm
lambda = -1;        %Eigenvalue for the transfer function basis

alphaTol = 5e-4;    %Tolerance for the bisection algorithm

%% Structurization
constants.lambda        = lambda;
constants.dB            = dB;
constants.SDPoptions    = SDPoptions;
constants.LMIsolver     = LMIsolver;
constants.radius        = radius;
constants.alphaTol      = alphaTol;

end