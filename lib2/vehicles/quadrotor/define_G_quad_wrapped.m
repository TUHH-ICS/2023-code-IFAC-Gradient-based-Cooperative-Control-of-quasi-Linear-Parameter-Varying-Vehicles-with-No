%---------------------------------------------------------------------------------------------------
% For Paper
% "Gradient-based Cooperative Control ofquasi-Linear Parameter Varying Vehicleswith Noisy Gradients"
% by Adwait Datar, Antonio Mendez Gonzalez and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
function [G]=define_G_quad_wrapped(dim,kp,kd)
% This function defines a closed-loop quadrotor model that has flocking
% force as input and it preserves the integral action

% Define double integrator to generate references
if dim~=2
    error('Currently just works with dimension=2')
end
%kd=3;
inv_mass=(1/0.640);
%kp=0.5;
A=kron([0 1; 0 -kd*inv_mass],eye(dim)); 
B=kron([0;1*inv_mass*kp],eye(dim));
C=eye(2*dim);
D=zeros(2*dim,dim);
G_double_int=ss(A,B,C,D);

G_quad_cl=get_quad_G_cl();
G=G_quad_cl*G_double_int;
% Compare the wrapped plant with the generic vehicle model
% sigma(G_double_int)
% hold on
% sigma(G,'r')
end