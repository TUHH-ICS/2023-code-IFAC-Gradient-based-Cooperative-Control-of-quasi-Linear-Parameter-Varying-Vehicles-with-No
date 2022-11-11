%---------------------------------------------------------------------------------------------------
% For Paper
% "Gradient-based Cooperative Control ofquasi-Linear Parameter Varying Vehicleswith Noisy Gradients"
% by Adwait Datar, Antonio Mendez Gonzalez and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Antonio Mendez Gonzalez
%---------------------------------------------------------------------------------------------------
% This function implements and solves the LMIs of the paper (see Lemma 7).

function [result] = feasibilityTest(system, constants)

%% Load constants
LMIsolver       = constants.LMIsolver;					%Solver to be used (only sdpt3 supported in this script)
nu              = constants.nu;							%order for ZF multiplier
delta           = constants.delta;						%noise level
alpha           = constants.alpha;						%convergence speed to estimate
m_sml           = constants.m_sml;						%m for f in S(m,L)
L_sml           = constants.L_sml;						%L for f in S(m,L)
lambda          = constants.lambda;						%Eigenvalue for the transfer function basis
dB              = constants.dB;         				%Bound for strict inequalities
SDPoptionsSDPT3 = constants.SDPoptions.sdpt3; 			%Constants for the solver
radius          = constants.radius;						%Yalmip: Radius constraint on all pimal variables ||x||<radius

%% system parameters
d           = system.d;		
Kp          = system.Kp;
Kd          = system.Kd;
G           = system.G;
G1          = system.G1;
G2          = system.G2;

%% Sizes of G

nx = size(G.A,1);           %number of states
[nC, nB] = size(G.D);       %numbr of inputs/outputs
nB0 = 2*d;                  %size for B0 (2*d)

%% Pi Multiplier definition

Av = (lambda )*diag(ones(1,nu)) + diag(ones(1,nu-1),-1);            %A_nu matrix

Bv = [ 1 ; zeros(nu-1,1)];                                          %B_nu matrix

s = tf('s');

Pi_nu = 1;
Rnu = 1;

for ii = 1:nu-1
    Pi_nu = [Pi_nu ; s^ii/(s-lambda)^(ii) ];                      %Pi_nu basis
    Rnu(ii+1,ii+1) = [ 1/sqrt(factorial(ii)) ];                     %R_nu diagonal matrix
end


[Anut, Bnut, Cnut, Dnut] = ssdata(minreal(ss(Pi_nu)));              %State space data of Pi_nu basis


Avalpha = Av - 2*alpha*eye(nu);                                     %Av alpha matrix

Api = mdiag(Avalpha, Avalpha);                                      %Api matrix

Bpi = [ -m_sml*Bv  Bv ;                                             %Bpi matrix
         L_sml*Bv -Bv ];

Cpi = [ zeros(1,nu) zeros(1,nu) ;                                   %Cpi matrix
        eye(nu)     zeros(nu)   ;
        zeros(1,nu) zeros(1,nu) ;
        zeros(nu)   eye(nu)     ];

Dpi = [ -m_sml         1           ;                                %Dpi matrix
         zeros(nu,1)   zeros(nu,1) ;
         L_sml        -1           ;
         zeros(nu,1)   zeros(nu,1) ];

PimLss = ss(Api,Bpi,Cpi,Dpi);                                       %Constuct Pi_mL state space
%% Solver Settings


yalmip('clear');                                                    %clear yalmip workspace


SDPoptions = sdpsettings('savesolveroutput', 1, ...
                         'savesolverinput' , 1, ...
                         'verbose'         , 1, ...
                         'solver'          , LMIsolver);            %yalmip setup

fieldsSolver = fieldnames(SDPoptionsSDPT3);                         %load tolerance values from previoulsy defined constants
for jj = 1:length(fieldsSolver)
    SDPoptions.sdpt3.(fieldsSolver{jj}) = SDPoptionsSDPT3.(fieldsSolver{jj});
end

SDPoptions.radius               = radius; 

X0 = sdpvar(nu*2*d+nx, nu*2*d+nx, 'symmetric');                     %Lyapunov matrix

X1 = sdpvar(nu-1, nu-1, 'symmetric');                               %matrix for positivity LMIs
X3 = sdpvar(nu-1, nu-1, 'symmetric');                               %matrix for positivity LMIs
lam = sdpvar(1);                                                    %lagrange multiplier

H = sdpvar(1);                                                      %scalar for positivity constraint
P1 = sdpvar(1, nu);                                                 %scalar for positivity constraint
P3 = sdpvar(1, nu);                                                 %scalar for positivity constraint
%% LMI Construction


M0 = [ -eye(d)   zeros(d)       ;
        zeros(d) delta^2*eye(d) ];                                  %M matrix

P = [  0            zeros(1,nu)  H          -P3          ;
       zeros(nu,1)  zeros(nu)   -P1'         zeros(nu)   ;
       H'          -P1           0           zeros(1,nu) ;
      -P3'          zeros(nu)    zeros(nu,1) zeros(nu)   ];         %Matrix P for P1, P3 and H

for ii = 1:2                                                        %For extreme points of the LPV system G

    if ii ==1
        Gii = G1;                                                   %Point 1
    else
        Gii = G2;                                                   %Point 2
    end

    Aii = Gii.A;                                                    %A of G
    Bii = Gii.B;                                                    %B of G
    Cii = Gii.C;                                                    %C of G

    ExtG.A = Aii;                                                   %Extended A of G
    ExtG.B = [ Bii Bii ];                                           %Extended B of G
    ExtG.C = [ Cii        ;
               zeros(d,nx) ;
               zeros(d,nx) ;
               zeros(d,nx)  ];                                      %Extended C of G
    ExtG.D = [ zeros(nC,nB) zeros(nC,nB) ;
               eye(d)       zeros(d,nB)  ;
               zeros(d,nB)  eye(d)       ;
               eye(d)       zeros(d,nB)  ];                         %Extended D of G

    ExtGss{ii} = ss(ExtG.A, ExtG.B, ExtG.C, ExtG.D);                %Define extended G

    %kronecker product of Pi_mL and Id
    krPimLss = ss(kron(PimLss.A, eye(d)), kron(PimLss.B, eye(d)), kron(PimLss.C, eye(d)), kron(PimLss.D, eye(d)));
    
    Sys0{ii} = mdiag(krPimLss, ss(eye(2*d)))*ExtGss{ii};            %Define Sys0 
end

for ii = 1:2                                                        %For extreme points of the LPV system G

    A0 = Sys0{ii}.A;                                                %Get all matrices of Sys0
    B0 = Sys0{ii}.B;
    C10 = Sys0{ii}.C(1:(1+nu+1+nu)*d,:);
    C20 = Sys0{ii}.C((1+nu+1+nu)*d+1:end,:);
    D10 = Sys0{ii}.D(1:(1+nu+1+nu)*d,:);
    D20 = Sys0{ii}.D((1+nu+1+nu)*d+1:end,:);

    LMI_1_A{ii} = [ A0'*X0 + X0*A0 + 2*alpha*X0 X0*B0      ;
                    B0'*X0                      zeros(nB0) ];       %left part of LMI1

    LMI_1_B{ii} = mdiag(kron(P,eye(d)), lam*M0);                    %center part of right part of LMI1

    LMI_1_C{ii} = [ C10 D10 ;
                    C20 D20 ];                                      %right part of right part of LMI1

    LMI_1{ii} = LMI_1_A{ii} + LMI_1_C{ii}'*LMI_1_B{ii}*LMI_1_C{ii}; %LMI1

end

LMI_2 = H + (P1 + P3)*inv(Av)*Bv;                                   %LMI2

LMI_3_A = [ zeros(nu-1)     X1              zeros(nu-1, nu) ;
            X1              zeros(nu-1)     zeros(nu-1, nu) ;
            zeros(nu, nu-1) zeros(nu, nu-1) diag(P1)        ];      %center part of LMI3

LMI_3_B = [ eye(nu-1) zeros(nu-1,1) ;
            Anut      Bnut          ;
            Rnu*Cnut  Rnu*Dnut      ];                              %right part of LMI3

LMI_4_A = [ zeros(nu-1)     X3              zeros(nu-1, nu) ;
            X3              zeros(nu-1)     zeros(nu-1, nu) ;
            zeros(nu, nu-1) zeros(nu, nu-1) diag(P3)        ];      %center part of LMI4

LMI_4_B = LMI_3_B;                                                  %right part of LMI4


LMI_3 = LMI_3_B'*LMI_3_A*LMI_3_B;                                   %LMI 3
LMI_4 = LMI_4_B'*LMI_4_A*LMI_4_B;                                   %LMI 4


%% Constraints

Constraints = [];
Constraints = [ Constraints LMI_1{1} <= -dB*eye(size(LMI_1{1})) ];  %LMI1 <= 0 LPV point 1
Constraints = [ Constraints LMI_1{2} <= -dB*eye(size(LMI_1{2})) ];  %LMI1 <= 0 LPV point 2
Constraints = [ Constraints LMI_2 >= dB*eye(size(LMI_2)) ];         %LMI2 >= 0
Constraints = [ Constraints LMI_3 >= dB*eye(size(LMI_3)) ];         %LMI3 > 0
Constraints = [ Constraints LMI_4 >= dB*eye(size(LMI_4)) ];         %LMI4 > 0
Constraints = [ Constraints  X0 >= dB*eye(size(X0)) ];              %Lyap Matrix > 0
Constratins = [ Constraints lam >= dB];                             %lambda >= 0

Optimization = [];

Constraints                                                         %Display constraints

%% Feasibility Test

tic
solution  = solvesdp(Constraints, Optimization, SDPoptions)         %Run solver
toc


if startsWith(solution.info,'Successfully solved')                  %Categorize solution
    feas = 1;
elseif startsWith(solution.info,'Infeasible problem')
    feas = 0;
else
    feas = 2;
end

result.alpha = alpha;                                               %store results
result.feas  = feas;

end