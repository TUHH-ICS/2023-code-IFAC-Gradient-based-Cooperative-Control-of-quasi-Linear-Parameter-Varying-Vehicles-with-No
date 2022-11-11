%---------------------------------------------------------------------------------------------------
% For Paper
% "Gradient-based Cooperative Control ofquasi-Linear Parameter Varying Vehicleswith Noisy Gradients"
% by Adwait Datar, Antonio Mendez Gonzalez and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Antonio Mendez Gonzalez
%---------------------------------------------------------------------------------------------------
% This script plots the data for figure 7

figure(700)
close 700
figure(700)

textlegend = {};

%% Plot curves from simulation

plot(sim1_unstable.tout, sim1_unstable.x,'b');      %1st scenarion
grid on
hold on
plot(sim2_stable.tout, sim2_stable.x,'r');          %2nd scenario
axis([0 15 248 252])

%% Decoration
textlegend{end+1} = ['K_d = 18.5, \delta = 0.5'];
textlegend{end+1} = ['K_d = 29.0, \delta = 0.5'];

legend(textlegend,'Location','NorthEastOutside')
ylabel('x(t)')
xlabel('t')
title(['Single agent locating the source (L_\psi = 70)'])