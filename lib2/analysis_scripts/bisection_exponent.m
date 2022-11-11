%---------------------------------------------------------------------------------------------------
% For Paper
% "Gradient-based Cooperative Control ofquasi-Linear Parameter Varying Vehicleswith Noisy Gradients"
% by Adwait Datar, Antonio Mendez Gonzalez and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------

function [alpha_best,P_ret]=bisection_exponent(G_veh,m,L,alpha_lims,tolerences,multiplier_class)
% This function runs a bisection algorithm for obtaining the best
% exponent alpha by calling the analysis routine for a fixed alpha
% multiple times.
%   G_veh               -> Plant model (typically of a vehicle)
%   m                   -> strong convexity parameter (lower bound)
%   L                   -> Lipschitz constant for gradients (upper bound)
%   alpha_lims          -> Initial search space for bisection
%   tolerences          -> tolerences for the optimization: bisection, LMI_tol, condition no tol 
%   multiplier_class    -> IQC multipliers such as Circle Criterion, Zames Falb, etc. 
  
    % Get tolernces
    cond_tol=tolerences.cond_tol;
    cvx_tol=tolerences.cvx_tol;
    bisect_tol=tolerences.bisect_tol;
    
    % Analyze for alpha=alpha_low
    switch multiplier_class.id
        case 1  % Circle Criterion
            [status,P_ret]=verify_exp_stab_CC(G_veh,alpha_lims(1),m,L,cond_tol,cvx_tol);
        case 2  % Full block circle criterion
            [status,P_ret]=verify_exp_stab_FBCC(G_veh,alpha_lims(1),m,L,cond_tol,cvx_tol);
        case 3  % Zames Falb Multipliers with CC
            [status,P_ret]=verify_exp_stab_ZF(G_veh,alpha_lims(1),m,L,cond_tol,cvx_tol);        
        case 6 % Zames Falb Multipliers with a basis for LTI systems
            rho=multiplier_class.rho;
            psi_order=multiplier_class.psi_order;
            odd_flag=multiplier_class.odd_flag;
            causal_flag=multiplier_class.causal_flag; % 1: causal, -1:anti-causal, 0:non-causal 
            [status,P_ret]=verify_exp_stab_ZF_basis(G_veh,alpha_lims(1),m,L,odd_flag,causal_flag,rho,psi_order,cond_tol,cvx_tol);
        case 7 % Zames Falb Multipliers with a basis for LPV systems
            rho=multiplier_class.rho;
            psi_order=multiplier_class.psi_order;
            odd_flag=multiplier_class.odd_flag;
            causal_flag=multiplier_class.causal_flag; % 1: causal, -1:anti-causal, 0:non-causal 
            [status,P_ret]=verify_exp_stab_ZF_basis_LPV_example(G_veh,alpha_lims(1),m,L,odd_flag,causal_flag,rho,psi_order,cond_tol,cvx_tol);
        
        %%%%%%%%%%%%% LMIs that include noise channels in the gradient%%%%%%%%%%%%%%%%%
        case 11  % Circle Criterion with noise
            [status,P_ret]=verify_exp_stab_with_noise_CC(G_veh,alpha_lims(1),m,L,multiplier_class.delta_1,cond_tol,cvx_tol);
            
        case 61 % Zames Falb Multipliers with a basis for LTI systems
            rho=multiplier_class.rho;
            psi_order=multiplier_class.psi_order;
            odd_flag=multiplier_class.odd_flag;
            causal_flag=multiplier_class.causal_flag; % 1: causal, -1:anti-causal, 0:non-causal 
            [status,P_ret]=verify_exp_stab_with_noise_ZF_basis(G_veh,alpha_lims(1),m,L,multiplier_class.delta_1,odd_flag,causal_flag,rho,psi_order,cond_tol,cvx_tol);
    end
    
    if ~status % return -1 and stop if Infeasible for alpha_low
        %error('Infeasible for alpha_low. Choose a smaller alpha_low')
        alpha_best=-1;
        return
    end
    
    % Analyze for alpha=alpha_high
    switch multiplier_class.id
        case 1  % Circle Criterion
            [status,P]=verify_exp_stab_CC(G_veh,alpha_lims(2),m,L,cond_tol,cvx_tol);
        case 2  % Full block circle criterion
            [status,P]=verify_exp_stab_FBCC(G_veh,alpha_lims(2),m,L,cond_tol,cvx_tol);
        case 3  % Zames Falb Multipliers with CC
            [status,P]=verify_exp_stab_ZF(G_veh,alpha_lims(2),m,L,cond_tol,cvx_tol);        
        case 6 % Zames Falb Multipliers with a basis for LTI systems
            [status,P]=verify_exp_stab_ZF_basis(G_veh,alpha_lims(2),m,L,odd_flag,causal_flag,rho,psi_order,cond_tol,cvx_tol);
        case 7 % Zames Falb Multipliers with a basis for LPV systems
            [status,P]=verify_exp_stab_ZF_basis_LPV_example(G_veh,alpha_lims(2),m,L,odd_flag,causal_flag,rho,psi_order,cond_tol,cvx_tol);
        
        %%%%%%%%%%%%% LMIs that include noise channels in the gradient%%%%%%%%%%%%%%%%%
        case 11  % Circle Criterion with noise
            [status,P]=verify_exp_stab_with_noise_CC(G_veh,alpha_lims(2),m,L,multiplier_class.delta_1,cond_tol,cvx_tol);
        case 61 % Zames Falb Multipliers with a basis for LTI systems
            [status,P]=verify_exp_stab_with_noise_ZF_basis(G_veh,alpha_lims(2),m,L,multiplier_class.delta_1,odd_flag,causal_flag,rho,psi_order,cond_tol,cvx_tol);
    end
    if status
        alpha_best=alpha_lims(2); % Return alpha_high if feasible
        P_ret=P;
    else        
        % Start the bisection and repeat until the limits are within bisect_tol
        while alpha_lims(2)-alpha_lims(1)>bisect_tol
            alpha_mid=mean(alpha_lims); % bisect the current limits           
            switch multiplier_class.id
                case 1  % Circle Criterion
                    [status,P]=verify_exp_stab_CC(G_veh,alpha_mid,m,L,cond_tol,cvx_tol);
                case 2  % Full block circle criterion
                    [status,P]=verify_exp_stab_FBCC(G_veh,alpha_mid,m,L,cond_tol,cvx_tol);
                case 3  % Zames Falb Multipliers
                    [status,P]=verify_exp_stab_ZF(G_veh,alpha_mid,m,L,cond_tol,cvx_tol);            
                case 6 % Zames Falb Multipliers with a basis for LTI systems
                    [status,P]=verify_exp_stab_ZF_basis(G_veh,alpha_mid,m,L,odd_flag,causal_flag,rho,psi_order,cond_tol,cvx_tol);
                case 7 % Zames Falb Multipliers with a basis for LPV systems
                    [status,P]=verify_exp_stab_ZF_basis_LPV_example(G_veh,alpha_mid,m,L,odd_flag,causal_flag,rho,psi_order,cond_tol,cvx_tol);
                
                %%%%%%%%%%%%% LMIs that include noise channels in the gradient%%%%%%%%%%%%%%%%%                
                case 11  % Circle Criterion with noise
                    [status,P]=verify_exp_stab_with_noise_CC(G_veh,alpha_mid,m,L,multiplier_class.delta_1,cond_tol,cvx_tol);
                case 61 % Zames Falb Multipliers with a basis for LTI systems
                    [status,P]=verify_exp_stab_with_noise_ZF_basis(G_veh,alpha_mid,m,L,multiplier_class.delta_1,odd_flag,causal_flag,rho,psi_order,cond_tol,cvx_tol);
            end
            if status % Choose the upper half as the new limits if feasible
                alpha_lims(1)=alpha_mid;
                P_ret=P;
            else % Choose the lower half as the new limits if infeasible
                alpha_lims(2)=alpha_mid;        
            end
        end
        alpha_best=alpha_lims(1);
    end
end