%---------------------------------------------------------------------------------------------------
% For Paper
% "Gradient-based Cooperative Control ofquasi-Linear Parameter Varying Vehicleswith Noisy Gradients"
% by Adwait Datar, Antonio Mendez Gonzalez and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Antonio Mendez Gonzalez
%---------------------------------------------------------------------------------------------------
% This script plots the data for figure 6

figure(600)
close 600
figure(600)

alpha = [];
counter = 1;
optKd = [];
textlegend = {};

%% Collect data
for iKd = 1:length(gridKd)                      %all Kd points
    for idelta = 1:length(gridDelta)            %all delta points
        Info = Data{counter}.results{end};      %Last bisection is the one we want
        if Info.feas == 1                       %If feasible
            alpha(iKd,idelta) = Info.alpha;     %feasible alpha is used
        else
            alpha(iKd,idelta) = 0;              %Else, alpha = 0.
        end
        counter = counter + 1;                  %next grid point
    end
end

%% Plot Data

for idelta = 1:length(gridDelta)                        %for all deltas
    plot(gridKd, alpha(:,idelta),'linewidth',2)         %plot delta curve
    hold on
    %plot(gridKd, alpha(:,idelta),'k*')                   %Uncomment to plot each point, as well as line below marked with (*)
    grid on
    [optKd(idelta,2), indxOpt] = max(alpha(:,idelta));  %find best alpha in the delta curve
    optKd(idelta,1) = gridKd(indxOpt);                  %find Kd corresponding to that alpha
    textlegend{end+1} = ['\delta = ' num2str(gridDelta(idelta))];
    %textlegend{end+1} = '';                             % (*)
end

%Plot Optimal Kd Curve
plot(optKd(:,1),optKd(:,2),'k','LineWidth',2)
textlegend{end+1} = '';
plot(optKd(:,1),optKd(:,2),'o','markeredgecolor',[0 0 0],'MarkerFaceColor',[1 1 1])
textlegend{end+1} = '';

%% Decoration
legend(textlegend,'Location','NorthEastOutside')
title(['L=' num2str(L_sml)])
xlabel('K_d')
ylabel('\alpha')
