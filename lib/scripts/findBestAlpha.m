%---------------------------------------------------------------------------------------------------
% For Paper
% "Gradient-based Cooperative Control ofquasi-Linear Parameter Varying Vehicleswith Noisy Gradients"
% by Adwait Datar, Antonio Mendez Gonzalez and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Antonio Mendez Gonzalez
%---------------------------------------------------------------------------------------------------
% This function bisects until the best alpha within tolerance is found

function [success, results] = findBestAlpha(system, constants)

Init = true;        %To indicate first iteration (special case)
results = {};       %to store results

rangeAlpha = [0 constants.alphaMax];    %Range for bisection
alphaTol = constants.alphaTol;          %Tolerance for feasible alpha

%Temp variables to store alpha at different stages of the bisection
newAlpha        = min(rangeAlpha);      %NA: New Alpha, Alpha to test. Bisection starts with lowest alpha, i.e. 0
lastTrueAlpha   = max(rangeAlpha);      %LTA: Last True Alpha assumed to be maximum alpha
lastFalseAlpha  = max(rangeAlpha);      %LFA: Last False Alpha assumed to be maximum alpha (irrelevant, but needed for initialization)

%loop until a break occurs, in either of the following scenarios:
%a) FAIL: First iteration and "infeasible" alpha, i.e. no further attempts are needed
%b) FAIL: It might occur that, the first iteration is not infeasible, but labeled as
%"numerical problems". The algorithm searches for a feasible alpha, but
%ends up again close to the lower limit of alpha after a while, i.e. ~0 still
%with "numerical problems".
%c) PASS: Feasible alpha within tolerance

% variable feas is used to evalute the type of solution thrown by the solver
% feas = 0 --> infeasible
% feas = 1 --> feasible
% feas = 2 --> other (e.g. Numerical problems o lack of progress)

while true

    constants.alpha = newAlpha;                             %Load new alpha to test

    result = feasibilityTest(system ,constants);            %Solver executes

    feas = result.feas;                                     %Type of solution

    results{end+1} = result;                                %Results are stored

    if Init                                                 %First iteration (special case)
        if feas == 0
            success = false;    
            break                                           %Search finishes, scenario a)
        else
            lastTrueAlpha = newAlpha;                       %Update of LTA
            newAlpha = max(rangeAlpha);                     %Update of NA, max in range
        end
    end

    if ~Init                                                %Nominal iterations (all excpet first)
        if feas == 0 || feas == 2                           %Not a feasible result
            lastFalseAlpha = newAlpha;                      %Updte of LFA
            newAlpha = mean([lastTrueAlpha lastFalseAlpha]);%NA = 0.5*(LTA + LFA)

            if newAlpha <= 1e-10                            %Fixed tolerance for scenario b)
                success = false;                           
                break                                       %Search finishes, scenario b)
            end
        else                                                %Feasible result
            if abs(lastTrueAlpha-newAlpha) < alphaTol       %Tolerance for scenario c)
                success = true;
                break                                       %Search finishes, scenario c)
            end

            lastTrueAlpha = newAlpha;                       %LTA = NA (feasible!)
            newAlpha = mean([lastTrueAlpha lastFalseAlpha]);%NA = 0.5*(LTA + LFA)

        end
    end
    Init = false;                                           %Not anymore 1st iteration
end

end