function [Z_dot,O_simulator,O_model] = esc_controller(t, Z,input)
%vehicle_simulator Simulator function that runs vehicle simulations
%   This is a wrapping function that is called by the numerical integrator.
%   It knows the current time-step and using it, it interpolates all the
%   inputs to be tracked. It also calculates the steering and throttle
%   control action needed and it passes these as scalar values to the
%   vehicle model.

%% Initialization : State Observer variables (only those required to calculate the necessary observer actions)

v_hat = Z(29); % Estimate of lateral velocity - v
r_hat = Z(30); % Estimate of yaq rate - r

%% Initialization : Controller Variables

% Assuming a simple PI Controller

%% Intialization : Low-Pass Filter (reference yaw rate)

Z_lpf = Z(31);

e_r_int = Z(32);


%% Initialization : Basic Vehicle Parameters
m = input.m_s;
Izz = input.J_z;
C1 = input.C1;
C2 = input.C2;
a = input.a_1;
b = abs(input.a_3);
l = a + b;
g = 9.81;

%% Initialization : Inputs (reference inputs to be tracked by controller)
delta_c = interp1(input.time, input.delta, t, 'pchip');

%% ---------------------------- ESC ALGORITHM -------------------------- %% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Control Action - Error calculation

% Target Yaw Rate (Filtered)
% NOTE - The FINAL target yaw rate is found by filtering an intermediate
% calculated yaw rate. This filtering is achieived by passing this
% intermediate yaw rate through a low pass filter (which is in state-space
% form). As a result of the integration, we already have access to the
% filtered yaw rate (as it is a state)
r_ref = input.C_lpf * Z_lpf;

e_r = r_hat - r_ref;

%% Control Action - P controller

kp = 1*4000;
ki = 1;
kd = 2;

m_d_c = -abs(kp * e_r + ki*e_r_int);
m_d_c_1 = 0;
m_d_c_2 = 0;
m_d_c_3 = 0;
m_d_c_4 = 0;
if e_r < 0 % Understeer since actual yaw rate is lower than requried yaw rate
    
    if r_hat > 0 % Left turn
        m_d_c_1 = 0.35 * m_d_c;
        m_d_c_2 = 0;
        m_d_c_3 = 0.5 * m_d_c;
        m_d_c_4 = 0.15 * m_d_c;
    elseif r_hat < 0 % Right turn
        m_d_c_1 = 0;
        m_d_c_2 = 0.35 * m_d_c;
        m_d_c_3 = 0.15 * m_d_c;
        m_d_c_4 = 0.5 * m_d_c;
    end

elseif e_r > 0 % Oversteer since actual yaw rate is higher than requried yaw rate 
    if r_hat > 0 % Left turn
        m_d_c_1 = 0;
        m_d_c_2 = 0.75 * m_d_c;
        m_d_c_3 = 0;
        m_d_c_4 = 0.25 * m_d_c;
    elseif r_hat < 0 % Right turn
        m_d_c_1 = 0;
        m_d_c_2 = 0.75 * m_d_c;
        m_d_c_3 = 0.25 * m_d_c;
        m_d_c_4 = 0;
    end

end

%% Measurement data 
% NOTE - Assuming that the four-wheel model's results can be assumed as a
% sensor data which will be fed to the state-estimator
q = Z(1:28);

[q_dot , ~ , ~ ,O_model] = vehicle_model_fw_simplified(q, input, delta_c, 0, m_d_c_1, m_d_c_2, m_d_c_3, m_d_c_4);

% Longitudinal Speed -Chassis Frame
u = O_model(1);

% Lateral Velocity - Chassis Frame
v_measured = O_model(2); % The lateral velocity state variable is in the world frame. Hence, inside the vehicle model, it is converted to the chassis frame and stored in the O_model vector

% Lateral Acceleration - Chassis Frame
ay_measured = O_model(3);
%% Target Yaw Rate
% Yaw Rate Reference
% There are 3 steps in generating the reference yaw rate. The reference yaw
% rate is generated with the purpose of maintaining stabliity of the
% vehicle rather than any performance requirements

% Step 0 : 
% Calculating vehicle behavioural properties
% Understeer gradient
eta = (m*9.81/l)*(b/C1 - a/C2); % Understeer Gradient
% Characteristic speed
v_ch = sqrt(9.81*l/eta); % Characteristic Speed
% Steady-state yaw rate response
ss_r_delta = (u/l)*(1/(1 + (u/v_ch)^2)); % Steady-state r/delta gain for constant speed

% Step 1:
% The first step calculates the target yaw rate based on the steady-state
% r/delta gain at the input speed. This is a linear gain 
r_r_1 = ss_r_delta*delta_c;

% This first step alone is INSUFFICIENT and extremely risky. As the
% target ay increases, the control input increases as well. As a result the
% slip angle of the front and rear increases as well. At a point the peak
% slip angle of the front and rear will pass. This is the negative
% cornering stiffness region of the vehicle. When we are in the negative
% cornering stiffness region of the rear, this is an extremely unstabl
% situation. When the vehicle goes in this region, the Fy produced by the
% tires will start to reduce so the output ay reduces which increases the
% error signal e_ay. As a result the ay controller increases its output.
% Correspondingly, since r_r_1 is only a linear gain, the reference yaw
% velocity also increases and so does the r controller's output which is
% the receipe for a catostraphic unstale system

% Step 2:
% In this step we limit or saturate the reference yaw rate 
ay_off = 1;
r_r_max = (abs(ay_measured) + ay_off)/u;
r_r_2 = min(max(r_r_1,-r_r_max),r_r_max); % This yaw rate needs to be filtered. Hence r_r_2 serves as the input to a low-pass filter


% Step 3:
% In this step we filter the reference yaw rate to behave like a
% first-order filter which has no response to high frequency inputs
% This is achieved by passing r_r_2 through a low-pass filter. 
% The low-pass filter can be written in a state-space format and the result
% of the integration is the filterd signal r_r_f = r_r_3. 

% ****** Extracted Above ******

% %% Measurement data 
% % NOTE - Assuming that the four-wheel model's results can be assumed as a
% % sensor data which will be fed to the state-estimator
% q = Q(1:28);
% [q_dot , ~ , ~ ,O_model] = vehicle_model_fw_simplified(q,input,delta_c,m_d_c);


%% ------------------------------ OBSERVER ----------------------------- %% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialization : Measured state

% This is the measured signal that the estimator tries to track
% This could be a signal coming from a sensor but in this code it is coming
% from the four-wheel model that is significantly more complex than the
% estimator model. Therefore, it can be considered to be sensor measurement

% ***** Extracted Above *****

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

%% Low Pass Filter Dynamics

U_lpf = r_r_2;
Z_dot_lpf = input.A_lpf*Z_lpf + input.B_lpf*U_lpf;

%% Augmented system dynamics ([model estimator])

Z_dot = [q_dot;
         (M_hat\f_hat_qd_q_u) + input.L*(y - y_hat);
         Z_dot_lpf;
         e_r
         ];

%% Initializing outputs to be logged

O_simulator = [O_model(2);
               r_ref;
               m_d_c;
               r_hat - ay_measured/u;
               m_d_c_1;
               m_d_c_2;
               m_d_c_3;
               m_d_c_4;
               ];








end