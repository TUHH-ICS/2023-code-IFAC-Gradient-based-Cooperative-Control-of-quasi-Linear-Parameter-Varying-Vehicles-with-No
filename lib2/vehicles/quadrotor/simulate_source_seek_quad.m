%---------------------------------------------------------------------------------------------------
% For Paper
% "Gradient-based Cooperative Control ofquasi-Linear Parameter Varying Vehicleswith Noisy Gradients"
% by Adwait Datar, Antonio Mendez Gonzalez and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
function [trajs]= simulate_source_seek_quad(G_veh,grad_field,time_steps,dt)
% This function simulates the feedback loop (G,grad)
    [A,B,C,D]=ssdata(G_veh);
    nx=size(A,1);
    [ny,nu]=size(D);
    x=zeros(nx,time_steps);
    y=zeros(ny,time_steps);
    u=zeros(ny,time_steps);    
    if D~=zeros(ny,nu)
        error('Vehicle model not strictly proper. (Necessary to avoid algebraic loop)')
    end
    % Initial condition
    %x(:,1)=[x_ic(1);x_ic(3);x_ic(2);x_ic(4);zeros(8,0)];    
    %y(:,1)=C*x_ic;
    for i=1:(time_steps-1)
        u(:,i)=-grad_field(y(:,i));
        x(:,i+1)=(eye(nx)+dt*A)*x(:,i)+dt*B*u(:,i);
        y(:,i+1)=C*x(:,i);
    end
    trajs=struct;
    trajs.x=x;
    trajs.y=y;
    trajs.u=u;
end