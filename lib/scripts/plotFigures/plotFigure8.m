%---------------------------------------------------------------------------------------------------
% For Paper
% "Gradient-based Cooperative Control ofquasi-Linear Parameter Varying Vehicleswith Noisy Gradients"
% by Adwait Datar, Antonio Mendez Gonzalez and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Antonio Mendez Gonzalez
%---------------------------------------------------------------------------------------------------
% This script plots the data for figure 8

figure(800)
close 800
figure(800)

%% Plot curves from simulation

plot(sim1_MAS.tout, sim1_MAS.x(:,1:10));
grid on
hold on
axis([0 5 245 255])

%% Decoration
ylabel('x(t)')
xlabel('t')
title(['10 agents locating the source (L_\psi = 66)'])