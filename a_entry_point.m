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

% Initializing the Low-Pass Filter
low_pass_filter;
%% CALCULATING OBSERVER GAIN MATRIX (L)

observer_gain;

%% INITIALIZATION
v_guess = input.u_start;
omega_y_1_guess = v_guess/input.r_01;
omega_y_2_guess = v_guess/input.r_02;
omega_y_3_guess = v_guess/input.r_03;
omega_y_4_guess = v_guess/input.r_04;

Z0 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 input.u_start 0 0 0 0 0 0 0 0 0 omega_y_1_guess omega_y_2_guess omega_y_3_guess omega_y_4_guess 0 0 0 0];

%% SIMULATION :- Simulation Options

% Initializing the options struct
opts = odeset('MaxStep',0.01);


%% SIMULATION :- RUN 
tic % Start timer

[t,z] = ode15s(@(t,q)esc_controller(t,q,input), [0 input.time(end)], Z0, opts); % Run simulation

timeTest=toc; % End timer


n_outputs_simulator = 8;
O_simulator = zeros(length(t),n_outputs_simulator);
parfor i=1:length(z)
    [~,O_simulator(i,:),~] = esc_controller(t(i),z(i,:)',input);
end


%% PLOTS

figure
plot(t, O_simulator(:,1), t, z(:,29))
legend("v","v_{hat}", Location="best")

% figure
% plot(t,(rad2deg(z(:,20))), t, rad2deg(z(:,30)))
% legend("r","r_{hat}", Location="best")

r_ref = O_simulator(:,2);
figure
plot(t, rad2deg(r_ref),'k');
hold on
plot(t, rad2deg(z(:,30)), 'b');
hold on 
plot(t, rad2deg(z(:,20)),'b--');
legend("r_{ref}", "r_{hat}", "r_{true}", Location="best")

m_d_c = O_simulator(:,3);
figure
subplot(3,2,[1 2])
plot(t,m_d_c);
legend("M_{d_c}")


m_d_c_1 = O_simulator(:,5);
subplot(3,2,3)
plot(t,m_d_c_1);

m_d_c_2 = O_simulator(:,6);
subplot(3,2,4)
plot(t,m_d_c_2);

m_d_c_3 = O_simulator(:,7);
subplot(3,2,5)
plot(t,m_d_c_3);

m_d_c_4 = O_simulator(:,8);
subplot(3,2,6)
plot(t,m_d_c_4);


beta_dot = O_simulator(:,4);
figure
plot(t, rad2deg(beta_dot));
legend("\beta_{dot}")
