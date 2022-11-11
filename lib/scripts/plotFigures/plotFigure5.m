%---------------------------------------------------------------------------------------------------
% For Paper
% "Gradient-based Cooperative Control ofquasi-Linear Parameter Varying Vehicleswith Noisy Gradients"
% by Adwait Datar, Antonio Mendez Gonzalez and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Antonio Mendez Gonzalez
%---------------------------------------------------------------------------------------------------
% This script plots the data for figure 5

figure(500)
close 500
figure(500)

counter = 1;
textlegend = {};

%% Plot data over all grid points
for i = 1:length(gridDelta)                     %Delta first
    for j = 1:length(gridL)                     %L second (following the same structure during the analysis)
        Info = Data{counter}.results{end};      %Last bisection is the one we want
        if Info.feas == 1                       %If feasible
            alpha(j) = Info.alpha;              %feasible alpha is used
        else
            alpha(j) = 0;                       %Else, alpha = 0.
        end
        counter = counter + 1;                  %next grid point
    end

    plot(gridL, alpha,'linewidth',2)            %Plot the delta curve 
    hold on
    %plot(gridL, alpha,'k*')                    %Uncomment to plot each point, as well as line below marked with (*)
    grid on
    textlegend{end+1} = ['\delta = ' num2str(gridDelta(i))];
    %textlegend{end+1} = '';                    % (*)

end

%% Decoration
legend(textlegend,'Location','NorthEastOutside')
xlabel('L')
ylabel('\alpha')
title(['Kd = ' num2str(Kd)])
