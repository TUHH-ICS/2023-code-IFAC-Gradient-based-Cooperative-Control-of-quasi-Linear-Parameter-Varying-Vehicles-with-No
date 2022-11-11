%---------------------------------------------------------------------------------------------------
% For Paper
% "Gradient-based Cooperative Control ofquasi-Linear Parameter Varying Vehicleswith Noisy Gradients"
% by Adwait Datar, Antonio Mendez Gonzalez and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
function G_quad_wrapped=get_quad_G_cl()
% This function defines the quadrotor model, designs a tracking controller
% and returns wrapped up closed loop of the quadrotor that takes as input
% positions and velocities and outputs quadrotor positions.
% -------------------------------------------------------------------------
    %% Define the model of the quadrocopter
    g = 9.81;   % Gravity constant
    m = 0.640;  %  Mass of the Quadrocopter
    A = [ 0  1  0  0  0  0  0  0  0  0  0  0   ;
          0  0  0  0  0  0  0  0 -g  0  0  0   ;
          0  0  0  1  0  0  0  0  0  0  0  0   ;
          0  0  0  0  0  0  0  0  0  0  g  0   ;
          0  0  0  0  0  1  0  0  0  0  0  0   ;
          0  0  0  0  0  0  0  0  0  0  0  0   ;
          0  0  0  0  0  0  0  1  0  0  0  0   ;
          0  0  0  0  0  0  0  0  0  0  0  0   ;
          0  0  0  0  0  0  0  0  0  1  0  0   ;
          0  0  0  0  0  0  0  0  0  0  0  0   ;
          0  0  0  0  0  0  0  0  0  0  0  1   ;
          0  0  0  0  0  0  0  0  0  0  0  0 ] ;
    
    % Note the transpose at the end of B
    B = [ 0  0  0  0  0 1/m 0  0  0  0  0  0   ;
          0  0  0  0  0  0  0  1  0  0  0  0   ;
          0  0  0  0  0  0  0  0  0  1  0  0   ;
          0  0  0  0  0  0  0  0  0  0  0  1 ]';
    C = eye(12);  
    D = zeros(12,4);
    P = ss(A,B,C,D);
    ref_dim=6; % Position and velocity references
    %% Design a tracking controller
    % Hinf design
    %G_quad_wrapped = hinf_design_2DOF_four_block(P,ref_dim);
    
    G_quad_wrapped = lqr_design_with_static_FF(P,ref_dim);
      
end