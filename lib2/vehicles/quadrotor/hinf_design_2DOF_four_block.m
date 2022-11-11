%---------------------------------------------------------------------------------------------------
% For Paper
% "Gradient-based Cooperative Control ofquasi-Linear Parameter Varying Vehicleswith Noisy Gradients"
% by Adwait Datar, Antonio Mendez Gonzalez and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
function G_quad_wrapped = hinf_design_2DOF_four_block(P,ref_dim)
% This function designs an hinf controller with 2DOF and fblock gp
    [A,B,C,D]=ssdata(P);    

    % Measure the sizes of all signals
    nu = size(B,2); % number of inputs
    ny = size(C,1); % number of outputs

    % Design 1
    % Ws=eye(ref_dim);
    % Wk=eye(nu);

    % Design 2
    ws=makeweight(1e4,5,1e-2);
    Ws=eye(ref_dim)*ws;
    Wk=0.1*eye(nu);


    % Compose the generalized plant
    systemnames = 'P Ws Wk';
    inputvar    = sprintf('[r(%d);d(%d);u(%d)]',ref_dim,nu,nu);
    input_to_P = '[u+d]';
    input_to_Ws = sprintf('[r-P(1:%d)]',ref_dim);
    input_to_Wk = '[u]';
    outputvar   = '[Ws; Wk; r;P]';
    cleanupsysic = 'yes';
    GP = sysic; % compose the defined setup

    NMEAS = ref_dim+ny;  % number of measured outputs
    NCON = nu;      % number of control inputs

    % Controller Synthesis
    [K,~,gam1,INFO] = hinfsyn(GP,NMEAS,NCON,'METHOD','LMI');
    
    % Close the loop with the feedback and feedforward components
    G_fb=feedback(P,-K(:,7:18));
    G_cl=G_fb*K(:,1:6);
    %% Wrap the closed loop such that it takes as input 
    % [x_des,y_des,xdot_des,ydot_des]
    % z_des and zdot_des are fixed to zero at the moment
    B_in=[1, 0, 0, 0;
          0, 0, 1, 0;
          0, 1, 0, 0;
          0, 0, 0, 1;
          0, 0, 0, 0;
          0, 0, 0, 0];
    C_out=zeros(2,12);
    C_out(1,1)=1;C_out(2,3)=1;
    G_quad_wrapped=C_out*G_cl*B_in;  
end