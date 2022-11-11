%---------------------------------------------------------------------------------------------------
% For Paper
% "Gradient-based Cooperative Control ofquasi-Linear Parameter Varying Vehicleswith Noisy Gradients"
% by Adwait Datar, Antonio Mendez Gonzalez and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
function G_quad_wrapped = lqr_design_with_static_FF(P,ref_dim)
% This function designs an LQR controller

    [A,B,~,~]=ssdata(P);   
    %LQR design    
    [nx,nu]=size(B);    
    Q=eye(nx);
    R=0.01*eye(nu);
    N=zeros(nx,nu);    
    [K,~,~] = lqr(P,Q,R);
    A_cl=(A-B*K);
    %% Obtain feedforward gain
    % z_des and zdot_des are fixed to zero at the moment
    C_ff=zeros(4,12);
    C_ff(1,1)=1;C_ff(2,3)=1;C_ff(3,2)=1;C_ff(4,4)=1;
    H1=-C_ff*inv(A_cl)*B;
    Kff=pinv(H1)*eye(4);    
    %% Wrap the closed loop such that it takes as inputs and outputs:
    % input: [x_des,y_des,xdot_des,ydot_des] and
    % output:[x,y]
    % z_des and zdot_des are fixed to zero at the moment        
    C_out=zeros(2,12);
    C_out(1,1)=1;C_out(2,3)=1;        
       
    G_quad_wrapped=ss(A_cl,B*Kff,C_out,0); 
end