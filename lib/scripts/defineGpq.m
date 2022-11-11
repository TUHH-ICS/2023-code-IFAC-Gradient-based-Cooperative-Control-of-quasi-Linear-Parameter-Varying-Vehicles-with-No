%---------------------------------------------------------------------------------------------------
% For Paper
% "Gradient-based Cooperative Control ofquasi-Linear Parameter Varying Vehicleswith Noisy Gradients"
% by Adwait Datar, Antonio Mendez Gonzalez and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Antonio Mendez Gonzalez
%---------------------------------------------------------------------------------------------------
% This function creates the Gpq system based on the number of dimensions
% and constants Kp and Kd

function [Gpq] = defineGpq(d,Kp,Kd)
%d:  number of spatial dimensions for each agent
%Kp: Kp gain of the Gpq system
%Kd: Kd gain of the Gpq system

%System matrices of the Gpq system
Apq = [ zeros(d)  eye(d)    ;
        zeros(d) -Kd*eye(d) ];

Bpq = [  zeros(d)  ;
        -Kp*eye(d) ];

Cpq = [ eye(d) zeros(d) ];

Dpq = zeros(d);

Gpq = ss(Apq, Bpq, Cpq, Dpq);

end