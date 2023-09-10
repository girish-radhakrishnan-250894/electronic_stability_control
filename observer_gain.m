%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%% OBSERVER GAIN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% -> This script calculates the observer gain matrix L using a linearized
%    version of the bicycle model 

%% Initialization : Estimator Parameters
% Parameters
m = input.m_s;
Izz = input.J_z;
C1 = input.C1;
C2 = input.C2;
u = input.u_start;
a = input.a_1;
b = abs(input.a_3);
l = a + b;
g = -9.81;

%% State Space Respresentation

% System Matrix
A = -[(C1 + C2)/(m*u),              u + (a*C1 - b*C2)/(m*u);
      (a*C1 - b*C2)/(Izz*u),    (a^2*C1 + b^2*C2)/(Izz*u)];

% Input Matrix
B = [C1/m;
     a*C1/Izz];

% Output Matrix
% NOTE - Assuming that lateral velocity is the output that can be measured 
C = [1 0];

%% State Observer Gain
% Using pole placement technique to create the observer gain matrix
% Using the principle of duality, (A-BK) is equivalent to (A' - C'L').
% Therefore, A', C' can be used using the placeMIMO function (which is the
% same as the place.m function of
input.L = placeMIMO(A',C',[-20,-20])';

eig(A - input.L*C)