# state_observer_four_wheel_model
Non-Linear state observer that estimates lateral velocity using measured signals from a four-wheel model

# RUNNING THE MODEL
a_entry_point.m                 -> As the name suggests, this is the entry script. Simply hitting PLAY inside this scrpt wil result running the observer simulation and plots the results
input_script.m                  -> To edit inputs
esc_controller.m                -> Wrapper function which is called by the ODE function. This function contains ESC algorithm. This function calls the core vehicle model from within
vehicle_model_fw_simplified.m   -> Core function containing the equations of motion of the vehicle model. This is function has only 1 role, accept some inputs and states, and calculate the forces and accelerations

# DESCRIPTION
This repository contains the control algorithm for an electronic stability 
controller AND a non-linear state observer. 
Apart from this, the repository also contains the algorithm of a 
four-wheel model. 

The goal of the ESC is to control the four-wheel model. It is known that 
the ESC provides stabiliy by controling the yaw rate with the goal of 
keeping beta_dot (rate of change of side-slip angle) small. Therefore, the 
state to be controlled is yaw rate. 

To utilize more of control theory and observre knowledge, I assume that
the yaw rate is not measureable and only lateral velocity is measureable. 

This gives an opportunity to implement a non-linear state estimator. 

The estimator is a non-linear bicycle model, the yaw rate is tracked and 
the lateral velocity is the state to be estimated. 
A linearized bicycle model along with pole placement technique is used to 
calculate the observer gains.