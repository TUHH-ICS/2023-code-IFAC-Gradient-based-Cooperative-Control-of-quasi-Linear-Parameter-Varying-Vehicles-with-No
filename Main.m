%---------------------------------------------------------------------------------------------------
% For Paper
% "Gradient-based Cooperative Control ofquasi-Linear Parameter Varying Vehicleswith Noisy Gradients"
% by Adwait Datar, Antonio Mendez Gonzalez and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Antonio Mendez Gonzalez
%---------------------------------------------------------------------------------------------------
% This script is used to generate data and plot the results for the
% Non-Linear Friction Vehicle Example (Section 5.2)
% This script is targeted only to replicate the figures of the paper.
% Extensions to other models are possible but not straighforward.
% Colors and labels may vary, as they are adjusted in tikz.

clc
clear all
close all

addpath(genpath(cd))

%% Setup
%Choose the Figure to be generated
%Following options are available for the variable Figure
% Figure = 0 --> Figures 5, 6, 7 and 8 in the paper.
% Figure = 5 --> Fig. 5 in paper
% Figure = 6 --> Fig. 6 in paper
% Figure = 7 --> Fig. 7 in paper
% Figure = 8 --> Fig. 8 in paper

%Choose RunAlgorithm:
% 0:Load stored data to generate figure
% 1:Run algorithm normally,

RunAlgorithm = 0;
Figure = 0;

%% Plot data

if RunAlgorithm == 1
    switch Figure
        case 0
            generateFig5
            generateFig6
            generateFig7
            generateFig8
        case 5
            generateFig5
        case 6
            generateFig6
        case 7
            generateFig7
        case 8
            generateFig8
        otherwise
            disp('Choose 0, 5, 6, 7 or 8. See comments in code.')
    end
else
    switch Figure
        case 0
            load Fig5.mat
            plotFigure5
            load Fig6.mat
            plotFigure6
            load Fig7.mat
            plotFigure7
            load Fig8.mat
            plotFigure8
        case 5
            load Fig5.mat
            plotFigure5
        case 6
            load Fig6.mat
            plotFigure6
        case 7
            load Fig7.mat
            plotFigure7
        case 8
            load Fig8.mat
            plotFigure8
        otherwise
            disp('Choose 0, 5, 6, 7 or 8. See comments in code.')
    end
end