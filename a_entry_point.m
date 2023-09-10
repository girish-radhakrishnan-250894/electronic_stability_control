%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%% TRIGGER SCRIPT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% -> This is the main script of this model and of this repository.

% -> This script is the first script that must be run.

% -> This script shall trigger all the remaining necessary scripts. 

% -> To edit the simulation inputs such as the steering angle, please 
%    navigate to the script called F02_INPUT_SCRIPT. 

% -> IMPORTANT :- The order in which the scripts are called below are very
%                 important. Changing the order will result in faulty
%                 simulation
clc;
clear;
addpath(genpath(pwd));

%% INPUT
% Running the script INPUT_SCRIPT initializes all the required mass,
% dimension, component and simulation inputs

input_script;

%% CALCULATING OBSERVER GAIN MATRIX (L)

observer_gain;

%% INITIALIZATION
v_guess = input.u_start;
omega_y_1_guess = v_guess/input.r_01;
omega_y_2_guess = v_guess/input.r_02;
omega_y_3_guess = v_guess/input.r_03;
omega_y_4_guess = v_guess/input.r_04;

q0 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 input.u_start 0 0 0 0 0 0 0 0 0 omega_y_1_guess omega_y_2_guess omega_y_3_guess omega_y_4_guess 0 0];

%% SIMULATION :- Simulation Options

% Initializing the options struct
opts = odeset('MaxStep',0.01);


%% SIMULATION :- RUN 
tic % Start timer

[t,q] = ode15s(@(t,q)state_observer(t,q,input), [0 input.time(end)], q0, opts); % Run simulation

timeTest=toc; % End timer


n_outputs_simulator = 1;
O_simulator = zeros(length(t),n_outputs_simulator);
parfor i=1:length(q)
    [~,O_simulator(i,:),~] = state_observer(t(i),q(i,:)',input);
end


%% PLOTS

figure
plot(t, O_simulator, t, q(:,29))
legend("v","v_{hat}", Location="best")

figure
plot(t,(rad2deg(q(:,20))), t, rad2deg(q(:,30)))
legend("r","r_{hat}", Location="best")
