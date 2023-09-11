# electronic_stability_control
 Develops ESC for a four-wheel model using a non-linear estimator

# RUNNING THE MODEL
a_entry_point.m                 -> As the name suggests, this is the entry script. Simply hitting PLAY inside this scrpt wil result running the observer simulation and plots the results
input_script.m                  -> To edit inputs
state_observer.m                -> Wrapper function which is called by the ODE function. This function contains observer algorithms. This function calls the core vehicle model from within
vehicle_model_fw_simplified.m   -> Core function containing the equations of motion of the vehicle model. This is function has only 1 role, accept some inputs and states, and calculate the forces and accelerations

