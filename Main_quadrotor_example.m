%---------------------------------------------------------------------------------------------------
% For Paper
% "Gradient-based Cooperative Control ofquasi-Linear Parameter Varying Vehicleswith Noisy Gradients"
% by Adwait Datar, Antonio Mendez Gonzalez and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
% This script is used to generate data and plot the results for the LTI
% quadrotor example presented in Fig.2 and Fig.3 in the above paper.  

close all
clear
clc
addpath(genpath('.\lib2'))

% Setup optimization

% Noise bounds
delta_1_all=0:0.1:0.9; % Multiplicative noise bound
for counter=1:size(delta_1_all,2)
    delta_1=delta_1_all(1,counter);

    % Sector bounds
    m=1; % lower bound on the sector
    L=10;  % Upper bound on the sector

    % Optimization tolerences
    tolerences.cvx_tol=1e-3; % Tolerence for definiteness in LMIs
    tolerences.bisect_tol=1e-3; % Tolerence in alpha for the bisect-algorithm 
    tolerences.cond_tol=1e8; % Tolerence for bounding the cond no of positive def variables

    % Pre-filter gains
    kp=1;
    kd=[1:1:30];

    % Run the analysis for different cases defined in the multiplier structure
    % with the following properties:

    % id: 
    % This determines the kind of multiplier used with the following choices
    %     1. Circle criterion
    %     6. Zames Falb multipliers with analysis LMIs for LTI systems
    %     7. Zames Falb multipliers with analysis LMIs for LPV systems

    % rho:
    % This is valid only for Zames Falb multipliers and is the pole location
    % for the basis functions parameterizing the multiplier

    % psi_order:
    % This is valid only for Zames Falb multipliers and is the order of the
    % multiplier that is being searched over

    % odd_flag:
    % This is valid only for Zames Falb multipliers and is set to one if the
    % non-linearity under consideration is odd and is set to 0 otherwise

    % causal_flag:
    % This is valid only for Zames Falb multipliers. It should be set to 1 if
    % restricting the search to causal multipliers, set to -1 is restricting
    % the search to anti-causal multipliers and set to 0 is searching over
    % general non-causal multipliers which includes causal and non-causal
    % parts.

    multiplier_flag=[61];
    for i=1:1    
        switch multiplier_flag(1,i)
            case 61  
                multiplier_class.id=61;
                multiplier_class.delta_1=delta_1;
                multiplier_class.rho=-1;
                multiplier_class.psi_order=1;
                multiplier_class.odd_flag=0;
                multiplier_class.causal_flag=0; % 1: causal, -1:anti-causal, 0:non-causal
                save_path=['.\lib2\data\mult_flag_noise_non_causal_',num2str(multiplier_flag(1,i)),'_noise_',num2str(counter-1)];  
        end    
        alpha_lims=[0,10]; % Initial range for the bisection algorithm
        [alpha_best]=sweep_kp_kd(m,L,kp,kd,alpha_lims,tolerences,multiplier_class);
        save(save_path);
    end
end
%% Plot data
plot_data
plot_examples
%% Functions
    function [alpha_best]=sweep_kp_kd(m,L,kp,kd,alpha_lims,tolerences,multiplier_class)
% This functions sweeps kp,kd and finds the best covergence rate estimate 
% by running a bisection algorithm for each fixed kp,kd
    n_p=length(kp);
    n_d=length(kd);
    alpha_best=zeros(n_p,n_d);   
    for i=1:n_p
        for j=1:n_d        
            kp_curr=kp(i);
            kd_curr=kd(j);
            % Quadrotor dynamics 
            % Current implementation only supports dim=2 for quadrotors
            dim=2;% spatial dimension (of positions and velocities)
            G_veh=define_G_quad_wrapped(dim,kp_curr,kd_curr); 
            [alpha_best(i,j),~]=bisection_exponent(G_veh,m,L,alpha_lims,tolerences,multiplier_class);        
        end        
    end   
end