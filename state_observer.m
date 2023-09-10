function [Q_dot,O_simulator,O_model] = state_observer(t, Q,input)
%vehicle_simulator Simulator function that runs vehicle simulations
%   This is a wrapping function that is called by the numerical integrator.
%   It knows the current time-step and using it, it interpolates all the
%   inputs to be tracked. It also calculates the steering and throttle
%   control action needed and it passes these as scalar values to the
%   vehicle model.

%% Initialization : State Observer variables (only those required to calculate the necessary observer actions)

v_hat = Q(29); % Estimate of lateral velocity - v
r_hat = Q(30); % Estimate of yaq rate - r

%% Initialization : Inputs (reference inputs to be tracked by controller)
delta_c = interp1(input.time, input.delta, t, 'pchip');

m_d_c = 0;

%% Measurement data 
% NOTE - Assuming that the four-wheel model's results can be assumed as a
% sensor data which will be fed to the state-estimator
q = Q(1:28);
[q_dot , ~ , ~ ,O_model] = vehicle_model_fw_simplified(q,input,delta_c,m_d_c);


%% Initialization : Measured state

% This is the measured signal that the estimator tries to track
% This could be a signal coming from a sensor but in this code it is coming
% from the four-wheel model that is significantly more complex than the
% estimator model. Therefore, it can be considered to be sensor measurement
r_measured = q_dot(6);

v_measured = O_model(2); % The lateral velocity state variable is in the world frame. Hence, inside the vehicle model, it is converted to the chassis frame and stored in the O_model vector

%% Initialization : State Observer 

% Parameters
m = input.m_s;
Izz = input.J_z;
C1 = input.C1;
C2 = input.C2;
u = O_model(1);
a = input.a_1;
b = abs(input.a_3);
l = a + b;
g = 9.81;
%% Observer Dynamics : Slip Angles

% Front slip angle
alpha_1 = -atan( ( -u*sin(delta_c) + (v_hat + a*r_hat)*cos(delta_c) ) / ( u*cos(delta_c) + (v_hat + a*r_hat)*sin(delta_c)  ));

% Rear slip angle
alpha_2 = -atan( ( -u*sin(0) + (v_hat - b*r_hat)*cos(0) ) / ( u*cos(0) + (v_hat - b*r_hat)*sin(0)  ));

%% Observer Dynamics : Forces

Fz_1 = m*g*b/(l);
Fz_2 = m*g*a/(l);

mf_input_1 = [Fz_1/2, 0, -alpha_1, 0, 0, u];
mf_input_2 = [Fz_2/2, 0, -alpha_2, 0, 0, u];

mf_output_1 = mfeval(input.tirFile_1, mf_input_1, 211);
mf_output_2 = mfeval(input.tirFile_2, mf_input_2, 211);

Fy_1__1 = mf_output_1(2)*2; % In Tire frame of refernecce 
Fy_2__2 = mf_output_2(2)*2; % In Tire frame of refernecce 

Fy_1 = Fy_1__1 * cos(delta_c); % In chassis frame of reference 
Fy_2 = Fy_2__2; % No steering in rear sore tire frame = chassis frame



%% Initializing : Measured output and estimator output

y = v_measured;

y_hat = v_hat;

%% Estimator Dynamics

% x_hat_dot = A*x_hat + B*u + L*(y - y_hat);

% Estimator Mass Matrix
M_hat = [m 0
         0 Izz];

% Estimator Force & Moments vector
f_hat_qd_q_u = [Fy_1 + Fy_2 - m*u*r_hat;
                a*Fy_1 - b*Fy_2
                ];
%% Augmented system dynamics ([model estimator])

Q_dot = [q_dot;
         (M_hat\f_hat_qd_q_u) + input.L*(y - y_hat)
         ];

%% Initializing outputs to be logged

O_simulator = [O_model(2)];








end